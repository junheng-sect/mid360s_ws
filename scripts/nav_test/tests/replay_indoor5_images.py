#!/usr/bin/env python3
"""Replay individual archived images through the real detector, fixed fit and QR service."""
import argparse
from pathlib import Path
from types import SimpleNamespace
import json
import os
import sys
import time
from unittest.mock import patch

import cv2
import numpy as np
import rosbag
import rospy
import rospkg
import yaml
from barcode_reader_camera.msg import Barcode
from geometry_msgs.msg import Point, Point32, Pose, PoseStamped, Quaternion
from ov13855_apriltag_vision.msg import TagCorners, TagCornerArray
from tf.transformations import quaternion_from_matrix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav_indoor4 import CAMERA_INFO, CAMERA_IMAGE, FRAME
from nav_indoor5 import Indoor5Runner, load_indoor5_route
from front_marker_control.localization import LocalPoseBuffer


def camera_info(path):
    c = yaml.safe_load(path.read_text())
    return CameraInfo(width=c['image_width'], height=c['image_height'], K=c['camera_matrix']['data'],
                      D=c['distortion_coefficients']['data'], distortion_model=c['distortion_model'])


def picture(pixels):
    height,width=pixels.shape
    return Image(width=width,height=height,encoding='mono8',step=width,data=pixels.tobytes())


class Replay:
    def __init__(self, record, folder, out):
        route=load_indoor5_route()
        args=SimpleNamespace(out=out, route=route, points=[(*p['xyz'],route['yaw_deg']) for p in route['points']],
                             check_segments=False, hold=3., delay=3., max_start_offset=float('inf'), preview=True)
        self.runner=Indoor5Runner(args,record['navigation_config'],None,None,None,None,None)
        self.runner.reference=np.asarray(record['transform_rotation']),np.asarray(record['transform_translation'])
        self.poses=LocalPoseBuffer(4096)
        with rosbag.Bag(str(next(folder.glob('*.bag')))) as bag:
            for _,m,_ in bag.read_messages(topics=['/drone_nav/odom']):
                p,q=m.pose.pose.position,m.pose.pose.orientation
                self.poses.accept(m.header.stamp.to_sec(),m.header.frame_id,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        config=Path(rospkg.RosPack().get_path('ov13855_cam'))/'config'
        self.small_info=camera_info(config/'ov13855_1280x960.yaml')
        self.full_info=camera_info(next(folder.glob('barcode/*/*/camera.yaml')))
        self.small=rospy.Publisher('/ov13855/image_raw',Image,queue_size=1)
        self.full=rospy.Publisher(CAMERA_IMAGE,Image,queue_size=1)
        self.si=rospy.Publisher('/ov13855/camera_info',CameraInfo,queue_size=1,latch=True)
        self.fi=rospy.Publisher(CAMERA_INFO,CameraInfo,queue_size=1,latch=True)
        self.active=None
        self.timer=rospy.Timer(rospy.Duration(1/15),self.publish)

    def publish(self,_):
        data=self.active
        if data is None:return
        small,full,pose=data
        header=Header(stamp=rospy.Time.now(),frame_id=FRAME)
        self.small_info.header=self.full_info.header=small.header=full.header=header
        # The archived vehicle attitude is interpolated at the original exposure.
        # A new ROS stamp keeps the repeated still usable in the real decoder retry window.
        odom=Odometry(header=Header(stamp=header.stamp-rospy.Duration(.08),frame_id='drone_odom'),child_frame_id='drone_nav_base')
        odom.pose.pose.position=Point(*pose.position)
        odom.pose.pose.orientation=Quaternion(*pose.quaternion)
        self.runner.on_odom(odom)
        self.si.publish(self.small_info);self.fi.publish(self.full_info)
        self.small.publish(small)
        if self.full.get_num_connections():self.full.publish(full)

    def inspect(self, path, stamp):
        img=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        assert img.shape==(3136,4224),path
        pose=self.poses.lookup(stamp,.2)
        assert pose is not None
        self.active=(picture(cv2.resize(img,(1280,960),interpolation=cv2.INTER_AREA)),picture(img),pose)
        # Discard buffered frames from the previous still (two additional exposure intervals).
        start=rospy.Time.now()+rospy.Duration(.2)
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            time.sleep(.08)
            frames=self.runner.vision.window(start,rospy.Time.now()-rospy.Duration(.25))
            if frames and self.runner.vision.ready_info():
                history=[]
                self.runner.observe([frames[-1]],history)
                return history
        raise AssertionError('no corners frame for '+str(path))

    def check_rejections(self, path, timestamp, target):
        runner=self.runner
        observed=self.poses.lookup(timestamp,.2)
        frame=runner.vision.latest
        tag=next(t for t in frame.tags if t.id==target)
        center,axes,_,_,_=runner.measure(tag,observed)
        basis=np.eye(4);basis[:3,:3]=axes
        pose=PoseStamped(header=frame.header,pose=Pose(Point(*center),Quaternion(*quaternion_from_matrix(basis))))
        meta=yaml.safe_load((path.parent/'meta.yaml').read_text())
        corners=np.asarray(meta['codes'][0]['corners'])
        assert runner.code_on_label(Barcode(format='QRCode',corners=corners.ravel()),pose)
        neighbour=corners+np.array([800.,0.])
        assert not runner.code_on_label(Barcode(format='QRCode',corners=neighbour.ravel()),pose)
        runner.vision.select(('tag36h11',target))
        with runner.vision.lock:
            runner.vision.forwarded.append(pose)
        assert runner.vision.matched_pose(frame.header.stamp+rospy.Duration(.011)) is None
        runner.vision.select(None)
        assert not runner.vision.forwarded and not runner.vision.pending

    def check_bad_corners(self):
        frame = self.runner.vision.latest
        key = frame.tags[0].id
        # Deliberately invalid input must stay reportable in the strict JSON mission record.
        tag = TagCorners(id=key, corners=[Point32(x=float('nan'), y=10.) for _ in range(4)])
        broken = TagCornerArray(header=frame.header, family=frame.family, tags=[tag])
        history = []
        self.runner.observe([broken],history)
        assert history[0]['corners_px'] is None and 'error' in history[0]
        json.dumps(history,allow_nan=False)

    def read(self, tag):
        # Exercise production select/decode/matched_pose/paper containment, using no flight action.
        with patch.object(self.runner,'stopped',return_value=None), patch.object(self.runner,'guard',return_value=None):
            return self.runner.decode_once(('tag36h11',tag))


def main(folder,out):
    assert os.environ['ROS_MASTER_URI']=='http://127.0.0.1:11448'
    rospy.init_node('historical_indoor5')
    record=json.loads(next(folder.glob('goal_test_indoor4*.json')).read_text())
    replay=Replay(record,folder,out)
    rospy.wait_for_service('/barcode_reader_camera/decode',10)
    rows=[]; ids=set()
    for b in record['batches']:
        photo=b['photo']
        hist=replay.inspect(folder/photo['file'],photo['stamp_ns']/1e9)
        valid=[h for h in hist if 'error' not in h];ids.update(h['id'] for h in valid)
        rows.append(dict(photo=photo['file'],fits=hist))
        print(photo['file'],[(h['id'],h.get('reprojection_rms_px',h.get('error'))) for h in hist],flush=True)
    assert ids=={0,1,2,3,4,5},ids
    for s in record['scans']:
        candidates=[]
        for path in folder.glob('barcode/*/*/meta.yaml'):
            meta=yaml.safe_load(path.read_text())
            for hit in meta['retries']:
                if abs(hit['stamp']-s['frame_ns']/1e9)<.000002:candidates.append((path.parent/hit['file'],hit['stamp']))
        (path,stamp),=candidates
        hist=replay.inspect(path,stamp)
        assert any(h['id']==s['id'] and 'error' not in h for h in hist)
        result=replay.read(s['id'])
        rows.append(dict(photo=str(path.relative_to(folder)),fits=hist,result=result))
        print(s['id'],result,flush=True)
        (out/'historical_results.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False))
        assert result['status']=='success' and result['qr']==s['qr'], result
        replay.check_rejections(path,stamp,s['id'])
    for samples in ({1:np.zeros(3),2:np.zeros(3)},
                    {1:np.zeros(3),2:np.zeros(3),3:np.array([.11,0.,0.])}):
        try:
            replay.runner.target(samples)
        except ValueError:
            pass
        else:
            raise AssertionError('insufficient or dispersed position samples accepted')
    replay.check_bad_corners()
    replay.timer.shutdown()
    print('PASS delayed odometry, neighbour QR rejection, 11 ms mismatch, target cache reset, sample count and position spread',flush=True)
    print('PASS six historical tag fits and three original QR payloads through production fixed-paper ROI',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('folder',type=Path);parser.add_argument('out',type=Path)
    args=parser.parse_args()
    main(args.folder,args.out)
