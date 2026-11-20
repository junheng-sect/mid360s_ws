#!/usr/bin/env python3
"""Real EGO/executor/mission/gateway, ideal tracking and synthetic observations.

Must run on its private localhost master via run_sequence_isolated.sh.
Never publishes /mavros/setpoint_raw/local. Body and inflation use flight values.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import actionlib
import numpy as np
import rosbag
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from mavros_msgs.msg import State, EstimatorStatus, PositionTarget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from traj_utils.msg import Bspline
from traj_utils.srv import CheckTrajectory
from drone_navigation.core import rz, wrap
from drone_navigation.msg import ExecuteRouteAction, ExecuteRouteGoal, NavigationCommand
from drone_navigation.ros_common import path_of

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav_sequence import SequenceRunner
from nav_square import SquareState
from analyze_sequence import analyze


class Fixture:
    def __init__(self, out, map_yaw=0.):
        self.map_yaw = map_yaw  # drone_map <- drone_odom; independent of the simulated PX4 alignment
        self.lock=threading.RLock();self.stop=False;self.preview=None
        self.p=np.array([0.,0.,1.]);self.a=math.radians(170)
        self.r=rz(.4);self.t=np.array([2.,-3.,.3]);self.bag=rosbag.Bag(str(out/'synthetic.bag'),'w')
        self.pubs={}
        for topic, typ in (
            ('/drone_nav/odom',Odometry),('/drone_nav/sensor_odom',Odometry),
            ('/drone_nav/cloud',PointCloud2),('/mavros/local_position/odom',Odometry),
            ('/mavros/state',State),('/mavros/estimator_status',EstimatorStatus),
            ('/drone_nav/input_status',String),('/drone_reloc/status',String),('/drone_reloc/result',String)):
            self.pubs[topic]=rospy.Publisher(topic,typ,queue_size=5,latch=topic.endswith('result'))
        self.tf=tf2_ros.TransformBroadcaster()
        for name, typ in (('command',NavigationCommand),('odom',Odometry),('bspline',Bspline),
                          ('executor_status',String),('gateway_status',String),('planner_status',String)):
            rospy.Subscriber('/drone_nav/'+name,typ,self.record,callback_args='/drone_nav/'+name,queue_size=500)
        rospy.Subscriber('/mavros/local_position/odom',Odometry,self.record,callback_args='/mavros/local_position/odom',queue_size=200)
        rospy.Subscriber('/drone_nav/setpoint_preview',PositionTarget,self.on_preview,queue_size=5)
        # Spherical far returns carve observed free rays. Each point is outside
        # the local range until approached; this is synthetic coverage only.
        k=np.arange(28000)+.5;z=1-2*k/len(k);rr=np.sqrt(1-z*z);phi=np.pi*(1+5**.5)*k
        dome=np.array([0,0,1])+8*np.c_[rr*np.cos(phi),rr*np.sin(phi),z]
        g=np.arange(-7,7.001,.12);x,y=np.meshgrid(g,g)
        self.points=np.r_[dome[dome[:,2]>-.2],np.c_[x.ravel(),y.ravel(),np.full(x.size,-.2)]]
        self.rng=np.random.RandomState(45)
        self.thread=threading.Thread(target=self.pump,daemon=True);self.thread.start()

    def record(self, msg, topic):
        with self.lock:
            if not self.stop:self.bag.write(topic,msg,rospy.Time.now())

    def on_preview(self, m):self.preview=m

    def pump(self):
        i=0
        while not self.stop and not rospy.is_shutdown():
            tick=time.monotonic();now=rospy.Time.now()
            if self.preview is not None:
                m=self.preview;self.p=self.r.T@(np.array([m.position.x,m.position.y,m.position.z])-self.t)
                self.a=wrap(m.yaw-.4);v=self.r.T@np.array([m.velocity.x,m.velocity.y,m.velocity.z])
            else:v=np.zeros(3)
            odom=Odometry();odom.header=Header(stamp=now,frame_id='drone_odom');odom.child_frame_id='drone_nav_base'
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=self.p
            odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=math.sin(self.a/2),math.cos(self.a/2)
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=rz(self.a).T@v
            self.pubs['/drone_nav/odom'].publish(odom)
            px=Odometry();px.header=Header(stamp=now,frame_id='px4_map');px.child_frame_id='drone_base'
            px.pose.pose.position.x,px.pose.pose.position.y,px.pose.pose.position.z=self.r@self.p+self.t
            px.pose.pose.orientation.z,px.pose.pose.orientation.w=math.sin((self.a+.4)/2),math.cos((self.a+.4)/2)
            self.pubs['/mavros/local_position/odom'].publish(px)
            state=State(connected=True,armed=True,mode='OFFBOARD');state.header.stamp=now
            self.pubs['/mavros/state'].publish(state)
            est=EstimatorStatus();est.header.stamp=now
            for key in ('attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag','pos_horiz_rel_status_flag','pos_vert_abs_status_flag'):setattr(est,key,True)
            self.pubs['/mavros/estimator_status'].publish(est)
            for topic,data in (('/drone_nav/input_status',dict(state='READY')),
                               ('/drone_reloc/status',dict(state='LOCALIZED',valid=True)),
                               ('/drone_reloc/result',dict(confirmed=True,map='synthetic'))):
                self.pubs[topic].publish(String(json.dumps(data)))
            tr=TransformStamped();tr.header=Header(stamp=now,frame_id='drone_map');tr.child_frame_id='drone_odom'
            tr.transform.rotation.z = math.sin(self.map_yaw / 2)
            tr.transform.rotation.w = math.cos(self.map_yaw / 2)
            self.tf.sendTransform(tr)
            if i%5==0:
                self.pubs['/drone_nav/sensor_odom'].publish(odom)
                points=self.points+self.rng.uniform(-.055,.055,self.points.shape)
                # Pack with NumPy: Python's per-point pack loop held the GIL
                # long enough to starve 50 Hz synthetic odometry on this board.
                data=points.astype('<f4').tobytes()
                cloud=PointCloud2(header=Header(stamp=now,frame_id='drone_odom'),height=1,width=len(points),
                    fields=[PointField(name=n,offset=j*4,datatype=PointField.FLOAT32,count=1) for j,n in enumerate(('x','y','z'))],
                    is_bigendian=False,point_step=12,row_step=12*len(points),data=data,is_dense=True)
                self.pubs['/drone_nav/cloud'].publish(cloud)
            i+=1;time.sleep(max(.001,.02-(time.monotonic()-tick)))

    def close(self):
        with self.lock:self.stop=True
        self.thread.join(timeout=3)
        with self.lock:self.bag.close()


def main():
    assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11439'
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
    rospy.init_node('sequence_isolated_fixture')
    f=Fixture(args.out)
    try:
        state=SquareState();buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf)
        rospy.wait_for_service('/drone_nav/prepare',timeout=20)
        prepare=rospy.ServiceProxy('/drone_nav/prepare',Trigger)
        check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
        client=actionlib.SimpleActionClient('/drone_nav/execute_route',ExecuteRouteAction)
        assert client.wait_for_server(rospy.Duration(15))
        for test in ('test4','test5'):
            runner=SequenceRunner(SimpleNamespace(test=test,preview=True,delay=3.,out=str(args.out)),
                rospy.get_param('/drone_nav/config'),state,buf,prepare,check,client)
            until=time.monotonic()+25
            while runner.inputs_problem() or not prepare().success:
                assert time.monotonic()<until,runner.inputs_problem()
                time.sleep(.1)
            # Fill synthetic ray sampling gaps without weakening the map check.
            goals,_,_=runner.make_route(f.p,f.a)
            until=time.monotonic()+20
            while True:
                result=check(path_of([f.p,goals[0]]))
                if result.valid:break
                assert time.monotonic()<until,result.reason
                time.sleep(.2)
            if test=='test4':
                # A bad later waypoint must reject the entire submitted route.
                sent=[]
                subscriber=rospy.Subscriber('/drone_nav/planner_goal',PoseStamped,lambda m:sent.append(m))
                poses=[]
                for p in (f.p, np.array([4.8,0.,1.])):
                    ps=PoseStamped();ps.header.frame_id='drone_map'
                    ps.pose.position.x,ps.pose.position.y,ps.pose.position.z=p
                    ps.pose.orientation.w=1.;poses.append(ps)
                client.send_goal(ExecuteRouteGoal(map_id='synthetic',waypoints=poses))
                assert client.wait_for_result(rospy.Duration(5))
                assert not client.get_result().success and 'clearance' in client.get_result().reason,client.get_result()
                assert not sent
                subscriber.unregister()
                print('PASS later out-of-margin waypoint rejected before first planner goal',flush=True)
            assert runner.preview(), 'sequence failed: '+test
            print('PASS real EGO sequence',test,flush=True)
        # A new 0.1 m translation must not be converted to a stationary turn.
        assert prepare().success
        time.sleep(.2)
        ps=PoseStamped();ps.header.frame_id='drone_map'
        ps.pose.position.x,ps.pose.position.y,ps.pose.position.z=f.p+np.array([.1,0,0])
        ps.pose.orientation.z,ps.pose.orientation.w=math.sin(f.a/2),math.cos(f.a/2)
        client.send_goal(ExecuteRouteGoal(map_id='synthetic',waypoints=[ps]))
        assert client.wait_for_result(rospy.Duration(8))
        assert not client.get_result().success
        print('PASS new short translation is not silently accepted as a turn',flush=True)
        publishers,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
        assert not any(topic=='/mavros/setpoint_raw/local' for topic,_ in publishers)
        print('PASS no real MAVROS setpoint publisher',flush=True)
    finally:
        f.close()
    results=[]
    for path in sorted(args.out.glob('goal_test*.json')):
        record=json.loads(path.read_text());summary=analyze(args.out/'synthetic.bag',record)
        (args.out/(record['test']+'_summary.json')).write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False))
        assert record['end']=='SUCCEEDED'
        for leg in summary['legs']:
            assert leg['command_v_peak_mps'] is not None
            assert leg['command_v_peak_mps']<=.5005
            assert leg['command_a_peak_mps2']<=.5005
            assert leg['premature_terminal_samples']==0,leg
            assert leg['command_continuity_excess_max_m']<=1e-6,leg
        if record['test']=='test4':
            for leg in summary['legs'][2:]:
                assert leg['replans']>0 and leg['intermediate_endpoints'],leg
        else:
            for leg in summary['legs'][4:]:
                assert 8.5<leg['turn_command_duration_s']<10.5,leg
                assert leg['motion_position_error_peak_m']<.02,leg
        results.append(summary)
    assert len(results)==2
    print('PASS intermediate trajectories, accepted replans, continuity, limits and four stationary 90deg turns',flush=True)
    rospy.signal_shutdown('done')


if __name__=='__main__':main()
