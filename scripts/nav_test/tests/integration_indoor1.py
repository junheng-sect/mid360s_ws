#!/usr/bin/env python3
"""Real EGO/mission/executor/gateway with synthetic ideal tracking on port 11441."""
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
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11441'
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--height', type=float, default=1., help='synthetic navigation-reference z, default 1 m')
    args = ap.parse_args()
    rospy.init_node('indoor1_isolated_fixture')
    cfg = rospy.get_param('/drone_nav/config')
    assert cfg['allow_unknown_above'] is False
    assert cfg['max_vel'] == cfg['max_acc'] == .5
    minimum = cfg['bounds_min'][2]+cfg['goal_boundary_margin']
    maximum = cfg['bounds_max'][2]-cfg['goal_boundary_margin']
    assert math.isfinite(args.height) and minimum <= args.height <= maximum
    fixture = Fixture(args.out)
    fixture.p = np.array([0., 0., args.height])
    records = []
    try:
        state, buf = SquareState(), tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buf)
        rospy.wait_for_service('/drone_nav/prepare', timeout=20)
        prepare = rospy.ServiceProxy('/drone_nav/prepare', Trigger)
        check = rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory)
        client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
        assert client.wait_for_server(rospy.Duration(15))
        assert abs(rospy.get_param('/drone_0_ego_planner_node/manager/max_vel')-.5/math.sqrt(3)) < 1e-9
        for index in (1, 2):
            runner = SequenceRunner(SimpleNamespace(test='test_indoor1', preview=True, delay=3., out=str(args.out)),
                                    cfg, state, buf, prepare, check, client)
            until = time.monotonic()+25
            while runner.inputs_problem() or not prepare().success:
                assert time.monotonic() < until, runner.inputs_problem()
                time.sleep(.1)
            # Cover sampling gaps by accumulating genuine synthetic rays,
            # leaving unknown-space rejection and flight inflation unchanged.
            until = time.monotonic()+20
            while True:
                goals, _, _ = runner.make_route(fixture.p.copy(), fixture.a)
                response = check(path_of([fixture.p.copy(), goals[0]]))
                if response.valid:
                    break
                assert time.monotonic() < until, response.reason
                time.sleep(.2)
            assert runner.preview(), 'indoor route failed'
            record = runner.record
            assert record['end'] == 'SUCCEEDED' and len(record['legs']) == 1
            assert record['real_output'] is False
            assert record['legs'][0]['hold_elapsed_s'] >= 5.
            p0 = np.array(record['start_odom'])
            a0 = math.radians(record['start_yaw_deg'])
            forward = np.array([math.cos(a0), math.sin(a0), 0.])
            np.testing.assert_allclose(np.array(record['goals_odom'])[0]-p0, forward, atol=1e-10)
            delta = fixture.p-p0
            assert .85 <= delta@forward <= 1.15, delta
            assert np.linalg.norm(delta-forward*(delta@forward)) < .05, delta
            assert abs(wrap(fixture.a-a0)) < math.radians(5)
            if records:
                np.testing.assert_allclose(p0, records[-1]['legs'][0]['end_odom'], atol=.03)
            records.append(record)
            print('PASS indoor forward iteration %d, full flight envelope and 5s hold' % index, flush=True)
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        assert not any(topic == '/mavros/setpoint_raw/local' for topic, _ in publishers)
        print('PASS no real MAVROS setpoint publisher', flush=True)
    finally:
        fixture.close()
    summaries = []
    for record in records:
        summary = analyze(args.out/'synthetic.bag', record)
        leg = summary['legs'][0]
        assert leg['command_v_peak_mps'] is not None
        assert leg['command_v_peak_mps'] <= .5005, leg
        assert leg['command_a_peak_mps2'] <= .5005, leg
        assert leg['end_error_m'] <= .15, leg
        assert abs(leg['end_height_error_m']) <= .05, leg
        assert abs(leg['end_yaw_error_deg']) <= 5., leg
        assert leg['premature_terminal_samples'] == 0, leg
        assert leg['command_continuity_excess_max_m'] <= 1e-6, leg
        assert not summary['faults'], summary['faults']
        summaries.append(summary)
    (args.out/'summary.json').write_text(json.dumps(summaries, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print('PASS limits, endpoint, height/yaw, command continuity and fault-free execution', flush=True)
    rospy.signal_shutdown('done')


if __name__ == '__main__':
    main()
