"""Isolated indoor5 mission, with image-plane corners and independent dispatch assertions."""
import time
import math
from unittest.mock import patch

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from traj_utils.srv import CheckTrajectory
from ov13855_apriltag_vision.msg import TagCorners, TagCornerArray
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

from integration_indoor4 import Camera, check_dispatched_goals, main
from nav_forward import path_of
from nav_indoor4 import FRAME, SIZE
from nav_indoor5 import Indoor5Runner, load_indoor5_route
from nav_square import Stopped
from drone_navigation.core import rz


class CornerCamera(Camera):
    tag_topic, tag_type = '/ov13855_apriltag_vision/corners', TagCornerArray
    target_topic = '/indoor5/target_tag_pose'
    missing_key, neighbor_key = None, None

    def configure(self):
        self.coords = {5:[.89,-1.31,.73], 1:[.23,-1.31,1.06], 2:[.98,-1.32,1.63],
                       0:[-.41,-1.30,1.10], 4:[-.43,-1.33,1.58], 3:[-.40,-1.36,.67]}
        self.coords = {key:np.array(p) for key,p in self.coords.items()}
        # At yaw -90 the paper's printed right points west. Rotate this reference by +0.5 deg.
        axes = rz(math.radians(.5)) @ np.array([[-1.,0,0],[0,0,1.],[0,1.,0]])
        self.tag_axes = {key:axes for key in self.coords}
        self.small_k = np.array([[945.,0,640.],[0,954.,480.],[0,0,1.]])
        self.dist = np.array([.10,-.02,.002,0.,0.])
        self.obj = self.layout['target_marker_size_m']*.5*np.array([[-1.,-1.,0],[1.,-1.,0],[1.,1.,0],[-1.,1.,0]])
        self.small = rospy.Publisher('/ov13855/camera_info',CameraInfo,queue_size=1,latch=True)

    def publish(self, _):
        p = rz(self.flight.map_yaw) @ self.flight.p.copy()
        body = rz(self.flight.a + self.flight.map_yaw)
        camera,view = p + body @ self.offset, body @ self.rotation
        header = Header(stamp=rospy.Time.now(),frame_id=FRAME)
        near = min(range(4),key=lambda i:np.linalg.norm(p-self.route[i]['xyz']))
        at_stop = np.linalg.norm(p-self.route[near]['xyz']) < .04
        ids = ((5,), (1,2,5), (0,1,4), (3,))[near] if at_stop else tuple(self.coords)
        tags = []
        for key in ids:
            pos = view.T @ (self.coords[key]-camera)
            if pos[2] <= .1:
                continue
            rvec = cv2.Rodrigues(view.T @ self.tag_axes[key])[0]
            px = cv2.projectPoints(self.obj,rvec,pos,self.small_k,self.dist)[0].reshape(4,2)
            tags.append(TagCorners(id=key,corners=[Point32(x=x,y=y) for x,y in px]))
        self.source = camera, view
        self.small.publish(CameraInfo(header=header,width=1280,height=960,distortion_model='plumb_bob',
                                      K=self.small_k.ravel().tolist(),D=self.dist.tolist()))
        self.tags.publish(TagCornerArray(header=header,family='tag36h11',width=1280,height=960,tags=tags))
        self.info.publish(CameraInfo(header=header,width=SIZE[0],height=SIZE[1],K=self.k,D=[0.]*5))


def unknown_footprint():
    # Establish fresh, narrowly observed space on the private grid before the dome fixture expands it.
    odom = rospy.Publisher('/drone_nav/sensor_odom', Odometry, queue_size=2)
    cloud = rospy.Publisher('/drone_nav/cloud', PointCloud2, queue_size=2)
    rospy.wait_for_service('/drone_nav/check_trajectory', 15.)
    check = rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory)
    deadline = time.monotonic()+10.
    try:
        while time.monotonic()<deadline:
            header=Header(stamp=rospy.Time.now(),frame_id='drone_odom')
            msg=Odometry(header=header)
            msg.pose.pose.position.z=.55
            msg.pose.pose.orientation.w=1.
            odom.publish(msg)
            cloud.publish(point_cloud2.create_cloud_xyz32(header,[[3.5,0.,.55],[3.5,.1,.65]]))
            time.sleep(.06)
            reply=check(path_of([np.array([-1.5,1.5,1.2])]))
            if not reply.valid and 'unknown' in reply.reason:
                print('PASS unknown scan footprint rejected:',reply.reason,flush=True)
                return
        raise AssertionError('expected unknown footprint on fresh partial grid: '+reply.reason)
    finally:
        odom.unregister()
        cloud.unregister()


def verify(runner, camera, messages, route, fixture, out):
    cfg = rospy.get_param('/drone_nav/config')
    assert (cfg['max_vel'], cfg['max_acc']) == (.5, .5)
    for section, keys in [('manager', ('max_vel', 'max_acc')), ('optimization', ('max_vel', 'max_acc')),
                          ('bspline', ('limit_vel', 'limit_acc'))]:
        actual = [rospy.get_param('/drone_0_ego_planner_node/%s/%s' % (section, key)) for key in keys]
        np.testing.assert_allclose(actual, [.4/math.sqrt(3)] * 2)
    print('PASS EGO 0.40 m/s and 0.40 m/s²; executor/gateway 0.50 m/s and 0.50 m/s²', flush=True)
    record=runner.record
    assert record['tag_counts']==dict(discovered=6,passed_precheck=6,arrived=6,read_ok=6), record['tag_counts']
    assert [b['order'] for b in record['batches']]==[[5],[2,1],[4,0],[3]], record['batches']
    check_dispatched_goals(messages,record,route,camera,out)
    assert all(s['status']=='success' and s['qr']=='label-%d'%s['id'] for s in record['scans'])
    assert all(len(o['corners_px'])==4 for b in record['batches'] for o in b['observations'])
    heights={c['id']:c['goal_height_map_m'] for b in record['batches'] for c in b['candidates'] if 'goal_height_map_m' in c}
    assert 1.43<heights[4]<heights[2]<1.70
    g=np.array([.62,-.04,1.70]);r,t=runner.reference
    runner.precheck(g,r.T@(g-t))
    above=g+[0,0,.0001]
    try:
        runner.precheck(above,r.T@(above-t))
    except ValueError as error:
        assert 'height limit' in str(error)
    else:
        raise AssertionError('accepted candidate above 1.70 m')
    for sample,exceeded in ((g,False),(above,True)):
        with patch.object(runner.s,'pose',return_value=(r.T@(sample-t),0.,0.)):
            try:
                runner.guard(stopped=True)
            except Stopped as error:
                assert exceeded and '1.70' in str(error), error
            else:
                assert not exceeded
    print('PASS 6/6 scan waypoints and payloads; ID 2/4 heights', heights, '1.70 m inclusive for candidates and actual pose',flush=True)


if __name__=='__main__':
    main(CornerCamera,Indoor5Runner,load_indoor5_route,verify,11447,unknown_footprint)
