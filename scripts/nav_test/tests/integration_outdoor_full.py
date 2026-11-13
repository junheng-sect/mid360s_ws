#!/usr/bin/env python3
"""test10 ROS integration: mock FCU/services/action on a fresh private master.

This tests the orchestration and real ROS delivery, NOT EGO dynamics, flight,
map clearance or real MAVROS. No setpoint publisher or arming service exists.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import actionlib
import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from traj_utils.srv import CheckTrajectory, CheckTrajectoryResponse
from drone_navigation.msg import ExecuteRouteAction, ExecuteRouteResult

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import nav_outdoor_full as nav


class ScenarioState(nav.FullState):
    """A new mock flight must not latch queued FCU packets from the previous scenario."""
    def __init__(self, since):
        self.since = since
        super().__init__()

    def on_fcu(self, message):
        if message.header.stamp >= self.since:
            super().on_fcu(message)


class Fixture:
    def __init__(self):
        self.lock = threading.RLock()
        self.closed = False
        self.runner = None
        self.map_id = 'synthetic_test10_only'
        self.publishers = {}
        for topic, typ in (
            ('/drone_nav/odom', Odometry), ('/mavros/local_position/odom', Odometry),
            ('/mavros/local_position/velocity_local', TwistStamped), ('/mavros/state', State),
            ('/mavros/extended_state', ExtendedState), ('/mavros/estimator_status', EstimatorStatus),
            ('/drone_nav/input_status', String), ('/drone_nav/executor_status', String),
            ('/drone_nav/gateway_status', String), ('/drone_reloc/status', String),
            ('/drone_reloc/result', String)):
            self.publishers[topic] = rospy.Publisher(topic, typ, queue_size=20)
        self.tf = tf2_ros.TransformBroadcaster()
        self.reset('normal')
        self.services = [rospy.Service('/drone_nav/prepare', Trigger, self.prepare),
                         rospy.Service('/drone_nav/prepare_takeoff', Trigger, self.prepare_takeoff),
                         rospy.Service('/drone_nav/takeoff', Trigger, self.takeoff),
                         rospy.Service('/drone_nav/check_trajectory', CheckTrajectory, self.check),
                         rospy.Service('/drone_nav/land', Trigger, self.land),
                         rospy.Service('/mavros/set_mode', SetMode, self.set_mode)]
        self.server = actionlib.SimpleActionServer('/drone_nav/execute_route', ExecuteRouteAction,
                                                   execute_cb=self.execute, auto_start=False)
        self.server.start()
        self.thread = threading.Thread(target=self.pump, daemon=True)
        self.thread.start()

    def reset(self, scenario):
        with self.lock:
            self.reset_stamp = rospy.Time.now()
            self.scenario = scenario
            self.position = np.array([.31 if scenario == 'bad_origin' else 0., 0., 0.])
            self.yaw = math.radians(11 if scenario == 'bad_yaw' else 0)
            self.translation = np.zeros(3)
            self.mode = 'ALTCTL' if scenario == 'startup_altctl' else 'POSCTL'
            self.armed, self.landed = scenario == 'startup_armed', 1
            self.gw, self.reason, self.exec_state, self.takeoff_state = 'ALIGNING', '', 'IDLE', ''
            self.input_state = 'READY'
            self.arm_timer = self.climb_timer = self.land_timer = None
            self.land_z0, self.land_state, self.land_count = 1., '', 0
            self.ready_timer = self.inject_timer = None
            self.mode_ready_timer = None
            self.stale_state = False
            self.requests, self.goals = [], []
            self.prepare_count = self.takeoff_count = 0
            self.check_count = 0
            self.runner = None

    def prepare(self, _):
        with self.lock:
            self.gw, self.reason = 'PREPARED', ''
            self.ready_timer = time.monotonic()+1.10
        return TriggerResponse(True, 'mock prepare')

    def prepare_takeoff(self, _):
        with self.lock:
            self.prepare_count += 1
            assert self.armed and self.landed == 1 and self.mode == 'POSCTL'
            self.gw, self.reason, self.takeoff_state = 'PREPARED', '', 'GROUND'
        return TriggerResponse(True, 'mock ground prepare')

    def takeoff(self, _):
        with self.lock:
            self.takeoff_count += 1
            assert self.mode == 'OFFBOARD' and self.landed == 1
            self.climb_timer = time.monotonic()
            self.takeoff_state = 'CLIMBING'
        return TriggerResponse(True, 'mock climb')

    def check(self, request):
        self.check_count += 1
        if self.scenario in ('future_unknown_new_view', 'future_unknown_persistent'):
            p = request.path.poses[0].pose.position
            target2 = p.x > 1.5 and p.y < -1.5 and p.z < 1.5
            if target2 and (self.scenario == 'future_unknown_persistent' or self.position[0] < 1.5):
                return CheckTrajectoryResponse(valid=False, reason='unknown footprint at 2.17,-2.10,.81; no observed voxel below')
        if self.scenario == 'map_unknown':
            return CheckTrajectoryResponse(valid=False, reason='unknown footprint at 0,-.3,.8; no observed voxel below')
        if self.scenario == 'map_occupied':
            return CheckTrajectoryResponse(valid=False, reason='occupied inflated footprint at 0,0,1')
        if self.scenario == 'service_timeout':
            time.sleep(1.)
        return CheckTrajectoryResponse(valid=True, reason='mock free space; not a real map proof')

    def set_mode(self, req):
        with self.lock:
            self.requests.append(dict(mode=req.custom_mode, at=time.monotonic()))
            if req.custom_mode != 'OFFBOARD':
                return SetModeResponse(mode_sent=False)
            self.mode, self.gw = 'OFFBOARD', 'READY'
        return SetModeResponse(mode_sent=True)

    def land(self, _):
        with self.lock:
            if self.scenario == 'land_refused':
                return TriggerResponse(False, 'mock gateway not READY')
            if self.gw != 'READY' or self.mode != 'OFFBOARD' or self.landed == 1:
                return TriggerResponse(False, 'mock gateway %s in %s' % (self.gw, self.mode))
            self.land_count += 1
            self.land_timer, self.land_z0 = time.monotonic(), float(self.position[2])
            self.land_state = 'DESCENDING'
        return TriggerResponse(True, 'mock descent at 0.1 m/s')

    def execute(self, goal):
        try:
            assert goal.map_id == self.map_id and len(goal.waypoints) == 1
            p = goal.waypoints[0]
            assert p.header.frame_id == 'drone_map'
            target = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
            with self.lock:
                self.goals.append(target.tolist())
                self.gw, self.reason, self.exec_state = 'READY', '', 'RUNNING'
                scenario, number = self.scenario, len(self.goals)
            start = time.monotonic()
            while not rospy.is_shutdown():
                if self.server.is_preempt_requested():
                    with self.lock:
                        self.gw, self.reason, self.exec_state = 'FAULT', nav.STOP_REASON, 'IDLE'
                    self.server.set_preempted(ExecuteRouteResult(success=False, reason='mock cancelled'))
                    return
                dt = time.monotonic()-start
                if scenario == 'leg_timeout':
                    time.sleep(.02)
                    continue
                if dt >= .12:
                    with self.lock:
                        self.position = target.copy()
                        self.exec_state = 'HOLD'
                        if number == 1 and scenario in ('takeover_recovers', 'input_recovers') and self.inject_timer is None:
                            self.inject_timer = time.monotonic()
                            if scenario == 'takeover_recovers':
                                self.mode = 'POSCTL'
                            else:
                                self.input_state = 'FAULT'
                if dt >= .45:
                    with self.lock:
                        self.yaw = math.atan2(2*p.pose.orientation.w*p.pose.orientation.z,
                                              1-2*p.pose.orientation.z**2)
                        self.gw, self.reason, self.exec_state = 'FAULT', nav.STOP_REASON, 'IDLE'
                    self.server.set_succeeded(ExecuteRouteResult(success=True, reason='mock settled'))
                    return
                time.sleep(.02)
        except Exception as e:
            self.server.set_aborted(ExecuteRouteResult(success=False, reason=str(e)))

    def pump(self):
        while not self.closed and not rospy.is_shutdown():
            with self.lock:
                now = time.monotonic()
                if self.mode_ready_timer is not None and now >= self.mode_ready_timer:
                    assert not self.requests and self.takeoff_count == 0 and not self.goals
                    self.mode, self.mode_ready_timer = 'POSCTL', None
                if self.runner is not None and self.runner.s.phase == 'WAIT_ARM' and self.arm_timer is None:
                    self.arm_timer = now+.15
                if self.arm_timer is not None and now >= self.arm_timer:
                    self.armed = True
                if self.ready_timer is not None and now >= self.ready_timer:
                    self.gw, self.ready_timer = 'READY', None
                if self.climb_timer is not None and self.runner is not None and self.runner.s.phase == 'TAKEOFF':
                    dt = now-self.climb_timer
                    if dt > .10:
                        self.landed, self.position[2] = 2, .82
                    if dt > .20:
                        self.takeoff_state = 'DONE'
                    if dt > .60 and self.scenario != 'done_low':
                        self.position[2] = 1.
                    if self.scenario == 'map_reset' and dt > .3:
                        self.translation[0] = .02
                    if self.scenario == 'stale_state':
                        self.stale_state = True
                if self.inject_timer is not None and now-self.inject_timer>.12:
                    self.mode, self.input_state = 'OFFBOARD', 'READY'
                if self.land_timer is not None:
                    self.position[2] = max(0., self.land_z0-.1*(now-self.land_timer))
                    if self.position[2] <= 0.:
                        self.land_state, self.landed = 'TOUCHED', 1
                        if self.scenario != 'land_no_disarm' and now-self.land_timer > 10.4:
                            self.armed = False
                self.publish()
            time.sleep(.02)

    def publish(self):
        stamp = rospy.Time.now()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'drone_odom', 'drone_nav_base'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.position
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        self.publishers['/drone_nav/odom'].publish(o)
        o.header.frame_id, o.child_frame_id = 'px4_map', 'drone_base'
        self.publishers['/mavros/local_position/odom'].publish(o)
        v = TwistStamped()
        v.header.stamp = stamp
        self.publishers['/mavros/local_position/velocity_local'].publish(v)
        if not self.stale_state:
            f = State(connected=True, armed=self.armed, mode=self.mode)
            f.header.stamp = stamp
            self.publishers['/mavros/state'].publish(f)
        ext = ExtendedState(landed_state=self.landed)
        ext.header.stamp = stamp
        self.publishers['/mavros/extended_state'].publish(ext)
        e = EstimatorStatus()
        e.header.stamp = stamp
        for k in ('attitude_status_flag', 'velocity_horiz_status_flag', 'velocity_vert_status_flag',
                  'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
            setattr(e, k, True)
        e.const_pos_mode_status_flag = self.landed == 1
        self.publishers['/mavros/estimator_status'].publish(e)
        for topic, d in (
            ('/drone_nav/input_status', dict(state=self.input_state, reason='mock fault')),
            ('/drone_nav/executor_status', dict(state=self.exec_state)),
            ('/drone_nav/gateway_status', dict(state=self.gw, reason=self.reason, real_output=True,
                                              takeoff=self.takeoff_state, land=self.land_state)),
            ('/drone_reloc/status', dict(state='LOCALIZED', valid=True)),
            ('/drone_reloc/result', dict(confirmed=True, map=self.map_id))):
            self.publishers[topic].publish(String(json.dumps(d)))
        tr = TransformStamped()
        tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp, 'drone_map', 'drone_odom'
        tr.transform.rotation.w = 1.
        tr.transform.translation.x = float(self.translation[0])
        self.tf.sendTransform(tr)

    def close(self):
        self.closed = True
        self.thread.join(3)


def wait_ready(runner):
    until = time.monotonic()+10
    while runner.inputs_problem() or runner.s.fcu is None or not runner.fc.ext_fresh() or \
            runner.fc.pos() is None or runner.fc.descent() is None or \
            (runner.fc.ground_z is None and not runner.s.initial_armed):
        assert time.monotonic() < until, runner.inputs_problem()
        time.sleep(.05)
    runner.transform()


def main():
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11443', 'private master required'
    p = argparse.ArgumentParser()
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    rospy.init_node('test10_mock_fixture')
    f = Fixture()
    results = []
    cfg = rospy.get_param('/drone_nav/config')
    cfg['map_id'] = f.map_id
    try:
        for scenario in ('normal', 'startup_altctl', 'future_unknown_new_view', 'future_unknown_persistent',
                         'done_low', 'bad_origin', 'bad_yaw', 'startup_armed', 'map_unknown', 'map_occupied',
                         'takeover_recovers', 'input_recovers', 'map_reset', 'stale_state',
                         'leg_timeout', 'land_refused', 'land_no_disarm', 'service_timeout'):
            f.reset(scenario)
            s = ScenarioState(f.reset_stamp)
            fc = nav.Fc(s)
            s.fc = fc
            buf = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buf)
            client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
            assert client.wait_for_server(rospy.Duration(10))
            r = nav.FullRunner(NS(preview=False, delay=1., out=str(a.out/scenario)), cfg, s, buf,
                               rospy.ServiceProxy('/drone_nav/prepare', Trigger),
                               rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory), client, fc,
                               rospy.ServiceProxy('/drone_nav/prepare_takeoff', Trigger),
                               rospy.ServiceProxy('/drone_nav/takeoff', Trigger),
                               rospy.ServiceProxy('/drone_nav/land', Trigger),
                               rospy.ServiceProxy('/mavros/set_mode', SetMode))
            wait_ready(r)
            f.runner = r
            if scenario == 'startup_altctl':
                with f.lock:
                    f.mode_ready_timer = time.monotonic()+.6
                nav.wait_ground_ready(r)
                assert not f.requests and not f.goals and f.takeoff_count == 0
            # Normal uses unmodified production timing; fault cases shorten only deadlines.
            with patch.object(nav, 'TAKEOFF_TIMEOUT', 4. if scenario != 'normal' else 20.), \
                    patch.object(nav, 'LEG_TIMEOUT', 1. if scenario == 'leg_timeout' else 20.), \
                    patch.object(nav, 'LAND_TIMEOUT', 14. if scenario == 'land_no_disarm' else 30.), \
                    patch.object(nav, 'PREFLIGHT_MAP_TIMEOUT', 1. if scenario == 'map_unknown' else 10.), \
                    patch.object(nav, 'LEG_COVERAGE_TIMEOUT', .8 if scenario == 'future_unknown_persistent' else 10.), \
                    patch.object(nav, 'SERVICE_TIMEOUT', .25 if scenario == 'service_timeout' else 2.):
                ok = r.run_once()
            assert ok == (scenario in ('normal', 'startup_altctl', 'future_unknown_new_view')), (scenario, r.flight)
            requests = [x['mode'] for x in f.requests]
            if scenario in ('normal', 'startup_altctl', 'future_unknown_new_view'):
                assert requests == ['OFFBOARD'] and len(f.goals) == 6 and f.land_count == 1
                np.testing.assert_allclose(f.goals, np.array(nav.ROUTE)[:,:3])
                assert r.flight['touchdown_disarmed']['landed'] == 1
                assert r.flight['touchdown_disarmed']['armed'] is False
                if scenario == 'future_unknown_new_view':
                    assert [x['target_index'] for x in r.flight['deferred_ground_targets']] == [2]
                    covered = [x for x in r.flight['leg_coverage_checks'] if x['leg'] == 2]
                    assert covered and covered[-1]['valid'] and covered[-1]['hold_odom'][0] > 1.5
            elif scenario == 'future_unknown_persistent':
                assert requests == ['OFFBOARD'] and len(f.goals) == 1 and f.land_count == 0
                assert '#2 目标包络等待' in r.flight['reason']
            elif scenario in ('bad_origin', 'bad_yaw', 'startup_armed', 'service_timeout', 'map_unknown', 'map_occupied'):
                assert requests == [] and f.takeoff_count == 0
                if scenario == 'map_unknown':
                    assert f.check_count >= 2 and '仍未通过' in r.flight['reason']
                if scenario == 'map_occupied':
                    assert f.check_count == 1
            elif scenario in ('done_low', 'map_reset', 'stale_state'):
                assert not f.goals and requests == ['OFFBOARD']
            elif scenario in ('takeover_recovers', 'input_recovers', 'leg_timeout'):
                assert len(f.goals) == 1 and requests == ['OFFBOARD']
            elif scenario == 'land_refused':
                assert requests == ['OFFBOARD'] and len(f.goals) == 6 and f.land_count == 0
            else:
                assert requests == ['OFFBOARD'] and len(f.goals) == 6 and f.land_count == 1
                if scenario == 'land_no_disarm':
                    assert '已确认着地但尚未上锁' in r.flight['reason']
            # No mode switch may be re-issued once the descent owns the vehicle.
            assert 'AUTO.LAND' not in requests
            # Keep publishing recovery briefly: the terminated state machine must stay inert.
            before = len(f.requests), len(f.goals)
            f.runner = None
            time.sleep(.20)
            assert before == (len(f.requests), len(f.goals))
            results.append(dict(scenario=scenario, passed=True, end=r.flight['end'],
                                reason=r.flight.get('reason', ''), requests=requests, goals=len(f.goals)))
            print('PASS mock ROS test10 ' + scenario, flush=True)
            del listener
        pubs, _, services = rosgraph.Master(rospy.get_name()).getSystemState()
        assert not any(t in ('/mavros/setpoint_raw/local', '/mavros/setpoint_position/local') for t,_ in pubs)
        assert not any(t == '/mavros/cmd/arming' for t,_ in services)
        assert dict(services)['/mavros/set_mode'] == [rospy.get_name()]
        print('PASS private mock FCU only; no arming service or real setpoint publisher', flush=True)
    finally:
        f.close()
        a.out.mkdir(parents=True, exist_ok=True)
        (a.out/'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))
        rospy.signal_shutdown('mock tests finished')


if __name__ == '__main__':
    main()
