#!/usr/bin/env python3
"""Real EGO/mission/executor/preview gateway, synthetic 0.5 m square at z=0.5 m."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import actionlib
import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction
from drone_navigation.core import wrap
from drone_navigation.ros_common import path_of

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integration_sequence import Fixture
from nav_sequence import SequenceRunner
from nav_square import SquareState
from analyze_sequence import analyze


def main():
    assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11442'
    ap=argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--height', type=float, default=.5)
    args=ap.parse_args()
    rospy.init_node('indoor2_isolated_fixture')
    cfg=rospy.get_param('/drone_nav/config')
    assert cfg['allow_unknown_above'] is False
    assert cfg['max_vel']==cfg['max_acc']==.5
    assert math.isfinite(args.height)
    assert cfg['bounds_min'][2]+cfg['goal_boundary_margin'] <= args.height <= cfg['bounds_max'][2]-cfg['goal_boundary_margin']
    fixture=Fixture(args.out)
    fixture.p=np.array([0.,0.,args.height])
    received=[]
    subscriber=rospy.Subscriber('/drone_nav/planner_goal',PoseStamped,lambda m:received.append(m))
    try:
        state,buf=SquareState(),tf2_ros.Buffer()
        listener=tf2_ros.TransformListener(buf)
        rospy.wait_for_service('/drone_nav/prepare',timeout=20)
        prepare=rospy.ServiceProxy('/drone_nav/prepare',Trigger)
        check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
        client=actionlib.SimpleActionClient('/drone_nav/execute_route',ExecuteRouteAction)
        assert client.wait_for_server(rospy.Duration(15))
        assert abs(rospy.get_param('/drone_0_ego_planner_node/manager/max_vel')-.5/math.sqrt(3))<1e-9
        assert rospy.get_param('/drone_0_ego_planner_node/grid_map/nav_radius')==.4
        assert rospy.get_param('/drone_0_ego_planner_node/grid_map/nav_half_height')==.3
        prepare_calls=[]
        def counted_prepare():
            prepare_calls.append(time.time())
            return prepare()
        runner=SequenceRunner(SimpleNamespace(test='test_indoor2',preview=True,delay=3.,out=str(args.out)),
                              cfg,state,buf,counted_prepare,check,client)
        until=time.monotonic()+25
        while runner.inputs_problem() or not prepare().success:
            assert time.monotonic()<until,runner.inputs_problem()
            time.sleep(.1)
        until=time.monotonic()+20
        while True:
            goals,_,_=runner.make_route(fixture.p.copy(),fixture.a)
            result=check(path_of([fixture.p.copy(),goals[0]]))
            if result.valid:
                break
            assert time.monotonic()<until,result.reason
            time.sleep(.2)
        assert runner.preview(), 'indoor square failed'
        record=runner.record
        assert record['end']=='SUCCEEDED' and len(record['legs'])==4
        assert record['real_output'] is False
        assert record['side_m']==.5 and record['hold_s']==2.
        assert len(prepare_calls)==4,prepare_calls
        assert len(received)==4,len(received)
        p0=np.array(record['start_odom']);yaw0=math.radians(record['start_yaw_deg'])
        f=np.array([math.cos(yaw0),math.sin(yaw0),0.]);right=np.array([math.sin(yaw0),-math.cos(yaw0),0.])
        goals=np.array(record['goals_odom'])
        np.testing.assert_allclose(np.diff(np.vstack([p0,goals]),axis=0),.5*np.array([f,right,-f,-right]),atol=1e-10)
        np.testing.assert_allclose(record['goal_yaw_odom_deg'],record['start_yaw_deg'])
        for i,leg in enumerate(record['legs']):
            assert leg['end']=='SUCCEEDED'
            assert 2. <= leg['hold_elapsed_s'] < 2.5,leg
            if i:
                assert record['legs'][i-1]['hold_end_wall']<=prepare_calls[i]<=leg['sent_wall'],leg
        assert np.linalg.norm(fixture.p-p0)<.03,fixture.p-p0
        assert abs(wrap(fixture.a-yaw0))<math.radians(5)
        goal_count=len(received)
        time.sleep(.5)
        assert len(received)==goal_count,'unexpected automatic repeat'
        publishers,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
        assert not any(topic=='/mavros/setpoint_raw/local' for topic,_ in publishers)
        (args.out/'checks.json').write_text(json.dumps(dict(prepare_calls_wall=prepare_calls,
            planner_goal_count=len(received),real_setpoint_publisher=False,
            closure_error_m=float(np.linalg.norm(fixture.p-p0))),indent=2)+'\n')
        print('PASS four 0.5m sides, fixed height/yaw, 2s holds, reprepare and closure',flush=True)
        print('PASS no extra goals and no real MAVROS setpoint publisher',flush=True)
    finally:
        subscriber.unregister()
        fixture.close()
    summary=analyze(args.out/'synthetic.bag',record)
    assert not summary['faults'],summary['faults']
    for leg in summary['legs']:
        assert leg['command_v_peak_mps'] is not None
        assert leg['command_v_peak_mps']<=.5005,leg
        assert leg['command_a_peak_mps2']<=.5005,leg
        assert leg['end_error_m']<=.15,leg
        assert abs(leg['end_height_error_m'])<=.05,leg
        assert abs(leg['end_yaw_error_deg'])<=5.,leg
        assert leg['premature_terminal_samples']==0,leg
        assert leg['command_continuity_excess_max_m']<=1e-6,leg
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print('PASS limits, endpoints, continuity and fault-free execution',flush=True)
    rospy.signal_shutdown('done')


if __name__=='__main__':
    main()
