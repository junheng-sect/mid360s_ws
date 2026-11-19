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
from indoor3_3_qr import paired_scan
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
        self.offsets = {'ov9281': np.array(cfg['camera_translation']) - lever,
                        'ov13855': np.array(cfg['upper_translation']) - lever}
        self.rot = rotation(cfg['camera_rotation_xyzw'])
        self.tag = np.array([.61,-1.15,.5])
        self.axes = np.array([[-1.,0,0],[0,0,1.],[0,1.,0]])
        self.visible = self.miss = self.drop = self.skew = self.hide_upper = False
        self.pubs = {}
        self.infos = {}
        self.tags = {}
        self.patches = {}
        self.rasters = {}
        self.start = {}
        for c, (topic, frame, w, h) in PHOTO_STREAMS.items():
            self.pubs[c] = rospy.Publisher(topic, Image, queue_size=1)
            self.tags[c] = rospy.Publisher('/%s_apriltag_vision/survey' % c, TagSurveyFrame, queue_size=4)
            info = rospy.Publisher(topic.replace('image_raw', 'camera_info'), CameraInfo, queue_size=1, latch=True)
            focal = 1500. if c == 'ov9281' else 3200.
            self.infos[c] = info, focal
            info.publish(CameraInfo(header=Header(stamp=rospy.Time.now(), frame_id=frame), width=w, height=h,
                                    distortion_model='plumb_bob', K=[focal,0,w/2,0,focal,h/2,0,0,1], D=[0]*5))
            self.patches[c] = {t: qr_patch(t, int(focal*.044/.64)) for t in ('BOX-99','BOX-01')}
            self.start[c] = None
        self.clock = rospy.Timer(rospy.Duration(.075),self.tick)

    def tick(self,_):
        body = rz(self.flight.a)
        view = body @ self.rot
        matrix = np.eye(4)
        matrix[:3,:3] = view.T @ self.axes
        for camera, pub in self.pubs.items():
            _,frame,w,h = PHOTO_STREAMS[camera]
            stamp = rospy.Time.now()
            pos = view.T @ (self.tag-self.flight.p-body@self.offsets[camera])
            visible = self.visible and not (camera=='ov13855' and self.hide_upper)
            tag_id = 1 if visible else 2
            pose = Pose(Point(*pos), Quaternion(*quaternion_from_matrix(matrix)))
            tag = TagObservation(id=tag_id, pose=pose, reprojection_rms_px=.3)
            skew = (.029 if self.skew else .00002) if camera == 'ov13855' else 0.
            self.tags[camera].publish(TagSurveyFrame(header=Header(stamp=stamp-rospy.Duration(skew),frame_id=frame),
                                                     family='tag36h11',ids=[tag_id],tags=[tag]))
            if not pub.get_num_connections():
                self.start[camera] = None
                continue
            if self.drop and camera=='ov9281':
                continue
            if self.start[camera] is None:
                self.start[camera] = time.monotonic()
            allow = not (self.miss and camera=='ov13855')
            text = 'BOX-01' if allow and time.monotonic()-self.start[camera]>.70 else 'BOX-99'
            patch = self.patches[camera][text]
            qr = pos + matrix[:3,1]*.063
            focal = self.infos[camera][1]
            x = int(round(w/2+focal*qr[0]/max(.01, qr[2])))-patch.shape[1]//2
            y = int(round(h/2+focal*qr[1]/max(.01, qr[2])))-patch.shape[0]//2
            # Reuse the raster during a hover so the fixture can also publish its 50 Hz odometry.
            key = (camera, text)
            if self.rasters.get(key, ())[:2] != (x,y):
                img = np.full((h,w),255,np.uint8)
                if qr[2] > 0 and 0<=x<=w-patch.shape[1] and 0<=y<=h-patch.shape[0]:
                    img[y:y+patch.shape[0],x:x+patch.shape[1]] = patch
                neighbour = self.patches[camera]['BOX-01']
                img[:neighbour.shape[0], :neighbour.shape[1]] = neighbour
                self.rasters[key] = x, y, img.tobytes()
            pub.publish(Image(header=Header(stamp=stamp,frame_id=frame), width=w,height=h,
                              step=w,encoding='mono8',data=self.rasters[key][2]))


def test_geometry(cam):
    ref = rz(.51), np.array([.3,-.7,.4])
    goals=[]
    for angle in (-.3,.3):
        axes=rz(angle)@cam.axes
        goal,center=fixed_scan_goal(cam.tag,axes,ref,cam.offsets["ov13855"],YAW)
        expected=cam.tag+.62*axes[:,2]
        np.testing.assert_allclose(center,expected)
        measured=goal+ref[0]@rz(heading(ref[0].T,YAW))@cam.offsets["ov13855"]
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
            paired_scan(folder,'BOX-01',guard,evidence)
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
    print('PASS OFFBOARD duplicate ignored, exit cancels active paired scan, next rising edge starts a new round')


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
        goal,_=fixed_scan_goal(cam.tag,cam.axes,(np.eye(3),np.zeros(3)),cam.offsets["ov13855"],YAW)
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
        assert outcomes['ov9281']['status']=='success' and outcomes['ov13855']['status']=='timeout', outcomes
        assert any('BOX-99' in attempt['texts'] for attempt in outcomes['ov9281']['reads'])
        assert all((runner.folder/s['photo']).is_file() for s in outcomes.values())
        assert outcomes['ov13855']['reason'] == 'roi_no_target'
        assert outcomes['ov9281']['strategy'] and outcomes['ov9281']['depth_m'] > 0.
        assert a['standoff_camera'] == 'ov13855' and a['standoff_m'] == .62
        cam.miss=False
        cam.axes=rz(.08)@cam.axes
        assert runner.preview(),runner.trial
        assert runner.trial['goal_yaw_map_deg']==a['goal_yaw_map_deg']==-89.5
        assert np.linalg.norm(np.array(a['control_goal_map'])-runner.trial['control_goal_map'])>.04
        assert all(s['status']=='success' for s in runner.trial['scan']['cameras'].values()),runner.trial
        assert np.linalg.norm(np.array(runner.record['start_odom'])-runner.record['goals_odom'][0])<.2
        reads = runner.trial['scan']['cameras']
        for camera, distance in [('ov13855', .62), ('ov9281', .65)]:
            assert abs(reads[camera]['range_m'] - distance) < .12, reads[camera]['range_m']
            assert 0 < reads[camera]['depth_m'] <= reads[camera]['range_m']
        for flag, reason in [('skew', 'no_matched_pose'), ('hide_upper', 'no_tag')]:
            setattr(cam, flag, True)
            assert runner.preview(), runner.trial
            scan = runner.trial['scan']['cameras']
            assert scan['ov13855']['reason'] == reason and scan['ov13855']['attempts'] == 0, scan
            assert scan['ov9281']['status'] == 'success'
            setattr(cam, flag, False)
        cam.drop=True
        assert not runner.preview() and runner.trial['status']=='stream_fault',runner.trial
        summary=json.loads((args.out/'success_rates.json').read_text())
        assert summary['completed']==4 and summary['cameras']['ov13855']['rate']==.25
        assert summary['cameras']['ov9281']['rate']==1.
        cam.drop=False
        edge_dir=args.out/'edge'
        edge_dir.mkdir()
        check_offboard_edges(edge_dir)
        with (args.out/'rounds.csv').open() as file:
            rounds = list(csv.DictReader(file))
            assert len(rounds) == 6 and rounds[0]['lock_tag1_frames'] == '0'
            assert rounds[1]['lock_outcome'] == 'locked' and int(rounds[1]['lock_max_window_frames']) >= 3
        assert not any(t=='/mavros/setpoint_raw/local' for t,_ in rosgraph.Master(rospy.get_name()).getSystemState()[0])
        print('PASS six rounds: no lock, paired ROI decode, neighbour then target, skew, missing tag, stream fault excluded')
    finally:
        cam.clock.shutdown()
        f.close()
        rospy.signal_shutdown('done')


if __name__=='__main__':
    main()
