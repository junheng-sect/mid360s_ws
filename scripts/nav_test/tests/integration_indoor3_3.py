#!/usr/bin/env python3
"""Private-master navigation, real QR pixels/ZXing and repeated trial evidence."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import actionlib
import cv2
import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion
from ov13855_apriltag_vision.msg import TagObservation, TagSurveyFrame
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_matrix
from traj_utils.srv import CheckTrajectory
from drone_navigation.core import rz, heading, rotation
from drone_navigation.msg import ExecuteRouteAction
from drone_navigation.ros_common import path_of

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from integration_sequence import Fixture
from indoor3_runner import PHOTO_STREAMS
from nav_indoor3_3 import Runner, YAW, fixed_scan_goal, settings
from nav_square import SquareState, Stopped
from indoor3_3_qr import ScanStream, scan_at_arrival
from contextlib import closing
from mavros_msgs.msg import State as FCUState
import threading


def qr_patch(text, size):
    png = subprocess.check_output(['qrencode', '-s', '4', '-m', '4', '-o', '-', text])
    qr = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
    return cv2.resize(qr, (size, size), interpolation=cv2.INTER_NEAREST)


class Camera:
    def __init__(self, flight, lever):
        self.flight = flight
        cfg = settings()
        self.offset = np.array(cfg['camera_translation']) - lever
        self.rot = rotation(cfg['camera_rotation_xyzw'])
        self.tag = np.array([.61,-1.15,.5])
        self.axes = np.array([[-1.,0,0],[0,0,1.],[0,1.,0]])
        self.visible = self.miss = self.drop = self.skew = self.hide = False
        self.arrival = None
        topic,self.frame,w,h = PHOTO_STREAMS['ov13855']
        self.pub = rospy.Publisher(topic, Image, queue_size=1)
        self.survey = rospy.Publisher('/ov13855_apriltag_vision/survey', TagSurveyFrame, queue_size=4)
        self.info = rospy.Publisher('/ov13855_full/camera_info', CameraInfo, queue_size=1, latch=True)
        self.focal = 3200.
        self.info.publish(CameraInfo(header=Header(stamp=rospy.Time.now(), frame_id=self.frame), width=w, height=h,
                                     distortion_model='plumb_bob', K=[self.focal,0,w/2,0,self.focal,h/2,0,0,1], D=[0]*5))
        self.patches = {t: qr_patch(t, round(self.focal*.044/.80)) for t in ('BOX-99','BOX-01')}
        self.rasters = {}
        self.clock = rospy.Timer(rospy.Duration(.075),self.tick)

    def tick(self,_):
        body = rz(self.flight.a)
        view = body @ self.rot
        matrix = np.eye(4)
        matrix[:3,:3] = view.T @ self.axes
        _,frame,w,h = PHOTO_STREAMS['ov13855']
        stamp = rospy.Time.now()
        pos = view.T @ (self.tag-self.flight.p-body@self.offset)
        tag_id = 1 if self.visible and not (self.hide and self.arrival is not None) else 2
        pose = Pose(Point(*pos), Quaternion(*quaternion_from_matrix(matrix)))
        tag = TagObservation(id=tag_id, pose=pose, reprojection_rms_px=.3)
        skew = .029 if self.skew else .00002
        self.survey.publish(TagSurveyFrame(header=Header(stamp=stamp-rospy.Duration(skew),frame_id=frame),
                                            family='tag36h11',ids=[tag_id],tags=[tag]))
        if not self.pub.get_num_connections() or self.drop:
            return
        # A genuine target before arrival must never short-circuit this round.
        correct = self.arrival is None or (not self.miss and time.monotonic()-self.arrival>.70)
        text = 'BOX-01' if correct else 'BOX-99'
        patch = self.patches[text]
        qr = pos + matrix[:3,1]*.063
        x = int(round(w/2+self.focal*qr[0]/max(.01, qr[2])))-patch.shape[1]//2
        y = int(round(h/2+self.focal*qr[1]/max(.01, qr[2])))-patch.shape[0]//2
        if self.rasters.get(text, ())[:2] != (x,y):
            img = np.full((h,w),255,np.uint8)
            if qr[2] > 0 and 0<=x<=w-patch.shape[1] and 0<=y<=h-patch.shape[0]:
                img[y:y+patch.shape[0],x:x+patch.shape[0]] = patch
            neighbour = self.patches['BOX-01']
            img[:neighbour.shape[0], :neighbour.shape[1]] = neighbour
            self.rasters[text] = x,y,img.tobytes()
        self.pub.publish(Image(header=Header(stamp=stamp,frame_id=frame), width=w,height=h,
                               step=w,encoding='mono8',data=self.rasters[text][2]))


def test_geometry(cam):
    ref = rz(.51), np.array([.3,-.7,.4])
    goals=[]
    for angle in (-.3,.3):
        axes=rz(angle)@cam.axes
        goal,center=fixed_scan_goal(cam.tag,axes,ref,cam.offset,YAW)
        expected=cam.tag+.80*axes[:,2]
        np.testing.assert_allclose(center,expected)
        measured=goal+ref[0]@rz(heading(ref[0].T,YAW))@cam.offset
        np.testing.assert_allclose(measured,expected)
        goals.append(goal)
    assert np.linalg.norm(goals[1]-goals[0])>.35
    print('PASS nonidentity map transform and angled normals change standoff position')


def check_offboard_edges(folder):
    # Drive the production edge/abort logic with private FCU messages. The
    # navigation fixture continues publishing preview only on the private master.
    state = object.__new__(SquareState)
    state.lock = threading.RLock()
    state.monitor = state.real = state.last_offboard = False
    state.entries = 0
    state.st = {}
    state.est = None
    def switch(mode):
        state.on_fcu(FCUState(connected=True, armed=True, mode=mode))

    switch('OFFBOARD')
    prepared = threading.Event()
    scanning = threading.Event()
    histories = []
    runner = object.__new__(Runner)
    runner.s = state
    def prepare():
        prepared.set()
        return SimpleNamespace(success=True, message='private test')
    runner.prepare = prepare
    class Finished(Exception):
        pass
    def lap():
        histories.append(state.entries)
        if len(histories) == 2:
            raise Finished()
        state.begin(True, 'synthetic')
        evidence = {}
        def guard():
            if 'start_ns' in evidence:
                scanning.set()
            if state.issue:
                raise Stopped(state.issue)
        try:
            with closing(ScanStream()) as stream:
                scan_at_arrival(stream, folder,'BOX-01',guard,evidence)
            raise AssertionError('scan survived OFFBOARD exit')
        except Stopped:
            assert 'OFFBOARD' in state.issue and scanning.is_set()
        finally:
            state.end()
        assert all(s['status']=='interrupted' for s in evidence['cameras'].values())

    runner.run_lap = lap
    def pilot():
        time.sleep(.15)
        switch('OFFBOARD')  # duplicate heartbeat must not start a round
        switch('POSCTL')
        assert prepared.wait(4)
        prepared.clear()
        switch('OFFBOARD')
        assert scanning.wait(7)
        switch('POSCTL')
        assert prepared.wait(4)
        switch('OFFBOARD')
    t=threading.Thread(target=pilot,daemon=True)
    t.start()
    # SquareRunner.automatic normally runs until stopped by the pilot.
    done=threading.Event()
    def schedule():
        try:
            runner.automatic()
        except Finished:
            done.set()
    scheduler=threading.Thread(target=schedule,daemon=True)
    scheduler.start()
    assert done.wait(16), 'OFFBOARD exit/re-entry did not rearm'
    t.join(1)
    assert histories==[2,3], histories
    print('PASS OFFBOARD duplicate ignored, exit cancels active scan, next rising edge starts a new round')


def main():
    assert os.environ['ROS_MASTER_URI']=='http://127.0.0.1:11448'
    ap=argparse.ArgumentParser(); ap.add_argument('--out',type=Path,required=True); args=ap.parse_args()
    rospy.init_node('indoor33_test')
    cfg=rospy.get_param('/drone_nav/config')
    f=Fixture(args.out)
    f.p=np.array([.45,.34,.50]); f.a=YAW+.1
    cam=Camera(f,np.array(cfg['lever_body']))
    try:
        test_geometry(cam)
        state,buf=SquareState(),tf2_ros.Buffer()
        listener=tf2_ros.TransformListener(buf)
        rospy.wait_for_service('/drone_nav/prepare',timeout=20)
        prepare=rospy.ServiceProxy('/drone_nav/prepare',Trigger)
        check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
        client=actionlib.SimpleActionClient('/drone_nav/execute_route',ExecuteRouteAction)
        assert client.wait_for_server(rospy.Duration(20))
        runner=Runner(SimpleNamespace(preview=True,out=str(args.out),delay=3.),cfg,state,buf,prepare,check,client)
        hold = runner.hold_at_point
        def at_arrival(*args):
            cam.arrival = time.monotonic()
            try:
                return hold(*args)
            finally:
                cam.arrival = None
        runner.hold_at_point = at_arrival
        goal,_=fixed_scan_goal(cam.tag,cam.axes,(np.eye(3),np.zeros(3)),cam.offset,YAW)
        until=time.monotonic()+30
        while runner.inputs_problem() or not prepare().success or not check(path_of([f.p,goal])).valid:
            assert time.monotonic()<until,'navigation not ready'
            time.sleep(.1)
        assert not runner.preview() and runner.trial['status']=='unlocked' and runner.record is None
        lock = runner.trial['lock']
        assert lock['outcome'] == 'timeout' and lock['survey_frames'] > 0 and lock['tag1_frames'] == 0
        cam.visible=True
        cam.miss=True
        assert runner.preview(),runner.trial
        a=runner.trial
        np.testing.assert_allclose(a['control_goal_map'],goal,atol=.01)
        assert abs(a['arrival_yaw_map_deg']+89.5)<=5 and len(a['tag_stamps_ns'])>=3
        lock = a['lock']
        assert lock['outcome'] == 'locked' and lock['pose_frames'] >= 3
        assert lock['windows'][-1]['stamps_ns'] == a['tag_stamps_ns']
        outcomes=a['scan']['cameras']
        assert set(outcomes)=={'ov13855'} and a['status']=='timeout', a
        missed=outcomes['ov13855']
        assert missed['status']=='timeout' and missed['reason']=='roi_no_target'
        assert any('BOX-99' in read['texts'] for read in missed['reads'])
        assert (runner.folder/missed['photo']).is_file()
        assert a['lock_camera']==a['standoff_camera']=='ov13855' and a['standoff_m']==.80
        assert a['scan']['end_ns']-a['scan']['start_ns']==2_000_000_000
        assert runner.record['legs'][0]['hold_elapsed_s'] >= 2.
        cam.miss=False
        cam.axes=rz(.08)@cam.axes
        assert runner.preview(),runner.trial
        assert runner.trial['goal_yaw_map_deg']==a['goal_yaw_map_deg']==-89.5
        assert np.linalg.norm(np.array(a['control_goal_map'])-runner.trial['control_goal_map'])>.04
        assert all(s['status']=='success' for s in runner.trial['scan']['cameras'].values()),runner.trial
        assert np.linalg.norm(np.array(runner.record['start_odom'])-runner.record['goals_odom'][0])<.2
        scan=runner.trial['scan']; result=scan['cameras']['ov13855']
        assert runner.trial['status']=='success' and 0 < result['first_match_ms'] < 2000
        assert any('BOX-99' in read['texts'] for read in result['reads'][:-1])
        assert 'BOX-01' in result['reads'][-1]['texts'] and result['photo_stamp_ns']==result['reads'][-1]['stamp_ns']
        assert abs(result['range_m']-.80)<.12 and abs(result['arrival_range_m']-.80)<.12
        assert 0 < result['depth_m'] <= result['range_m'] and result['strategy']
        for read in result['reads']:
            assert scan['start_ns']<=read['stamp_ns']<scan['end_ns']
            assert scan['start_ns']<=read['pose_stamp_ns']<scan['end_ns']
            assert abs(read['stamp_ns']-read['pose_stamp_ns'])<=10_000_000
        for flag, reason in [('skew', 'no_matched_pose'), ('hide', 'no_tag')]:
            setattr(cam, flag, True)
            assert runner.preview(), runner.trial
            result=runner.trial['scan']['cameras']['ov13855']
            assert result['reason']==reason and result['attempts']==0, result
            if flag=='skew': assert result['arrival_range_m']>0
            setattr(cam, flag, False)
        cam.drop=True
        assert not runner.preview() and runner.trial['status']=='stream_fault',runner.trial
        summary=json.loads((args.out/'success_rates.json').read_text())
        assert summary['completed']==4 and summary['cameras']['ov13855']['rate']==.25
        assert set(summary['cameras'])=={'ov13855'} and summary['cameras']['ov13855']['completed']==4
        cam.drop=False
        edge_dir=args.out/'edge'
        edge_dir.mkdir()
        check_offboard_edges(edge_dir)
        with (args.out/'rounds.csv').open() as file:
            rounds = list(csv.DictReader(file))
            assert len(rounds) == 6 and rounds[0]['lock_tag1_frames'] == '0'
            assert rounds[1]['lock_outcome'] == 'locked' and int(rounds[1]['lock_max_window_frames']) >= 3
        assert not any(t=='/mavros/setpoint_raw/local' for t,_ in rosgraph.Master(rospy.get_name()).getSystemState()[0])
        for group in rosgraph.Master(rospy.get_name()).getSystemState()[:2]:
            assert not any(t.startswith(('/usb_cam', '/ov9281')) for t,_ in group)
        print('PASS single-camera lock, timeout, early target, arrival distance, skew, missing tag and stream fault')
    finally:
        cam.clock.shutdown()
        f.close()
        rospy.signal_shutdown('done')


if __name__=='__main__':
    main()
