#!/usr/bin/env python3
"""离线测试：真实 ROS 消息类型 + 模拟时钟/action/状态，无 ROS master，无 MAVROS 发布。"""
import json
import math
import os
import sys
import tempfile
import time as wall_time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import EstimatorStatus
from std_msgs.msg import String

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 部署在 nav_test/tests/，也可与 nav_square.py 放在同一临时目录运行。
sys.path.insert(0, os.path.dirname(SCRIPT_DIR) if os.path.basename(SCRIPT_DIR) == 'tests' else SCRIPT_DIR)
import nav_forward
import nav_square as nav


class Clock:
    def __init__(self):
        self.t = 100.0
        self.hook = lambda: None
        self.strftime = wall_time.strftime

    def monotonic(self):
        return self.t

    def time(self):
        return 1000 + self.t

    def time_ns(self):
        return int(self.time() * 1e9)

    def sleep(self, seconds):
        self.t += seconds
        self.hook()


class Client:
    def __init__(self, h):
        self.h = h
        self.goals = []
        self.state = 3
        self.result = NS(success=True, reason='all waypoints settled')
        self.cancelled = 0
        self.finish_at = None

    def send_goal(self, goal):
        self.goals.append((self.h.clock.t, goal))
        self.state, self.result = 1, None
        self.finish_at = self.h.clock.t + .2
        self.h.gw, self.h.exec_state = 'READY', 'RUNNING'
        self.h.publish()

    def get_state(self):
        return self.state

    def get_result(self):
        return self.result

    def cancel_goal(self):
        self.cancelled += 1
        self.state, self.result = 2, NS(success=False, reason='cancelled')
        self.finish_at = None
        self.h.gw, self.h.reason, self.h.exec_state = 'FAULT', nav.STOP_REASON, 'IDLE'

    def wait_for_result(self, duration):
        return True


class Harness:
    # 子类换成本模块的 runner 即可复用整套假时钟/假 action/假 TF/遥测传送。
    RUNNER = nav.SquareRunner

    def __init__(self, preview=False):
        self.clock = Clock()
        self.preview = preview
        self.position = np.array([.2, -.1, 1.0])
        self.yaw = .37
        self.map_yaw = .6
        self.r = np.array([[math.cos(.6), -math.sin(.6), 0.], [math.sin(.6), math.cos(.6), 0.], [0., 0., 1.]])
        self.translation = np.array([5., -3., .4])
        self.gw, self.reason, self.exec_state = 'READY', '', 'IDLE'
        self.input_state, self.reloc_valid = 'READY', True
        self.offboard, self.connected = True, True
        self.ready_at = None
        self.check_valid = True
        self.prepare_calls, self.checked = [], []
        self.inject = lambda: None
        self.action_success = True
        self.stale = set()
        self.patches = [patch.object(nav, 'time', self.clock), patch.object(nav_forward, 'time', self.clock),
                        patch.object(rospy, 'Subscriber', return_value=None), patch.object(rospy, 'is_shutdown', return_value=False),
                        patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.clock.time())),
                        patch.object(nav, 'say', side_effect=lambda _: None)]
        for p in self.patches:
            p.start()
        self.state = nav.SquareState()
        self.tmp = tempfile.TemporaryDirectory(prefix='square-unit-')
        self.client = Client(self)
        self.args = self.make_args()
        self.cfg = dict(map_id='synthetic', bounds_min=[-3., -3., .4], bounds_max=[3., 3., 2.5], max_vel=.5, max_acc=.5)
        self.runner = self.RUNNER(self.args, self.cfg, self.state, self, self.prepare, self.check, self.client)
        self.clock.hook = self.tick
        self.publish()

    def make_args(self):
        return NS(preview=self.preview, delay=3.0, out=self.tmp.name)

    def close(self):
        self.tmp.cleanup()
        for p in reversed(self.patches):
            p.stop()

    def status(self, name, **data):
        self.state.on_status(String(json.dumps(data)), name)

    def publish(self):
        s = self.state
        if 'fcu' not in self.stale:
            s.on_fcu(NS(connected=self.connected, armed=True, mode='OFFBOARD' if self.offboard else 'POSCTL'))
        if 'odom' not in self.stale:
            o = Odometry()
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.position
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            s.on_odom(o)
        if 'input_status' not in self.stale:
            self.status('input_status', state=self.input_state, reason='')
        if 'reloc' not in self.stale:
            self.status('reloc', state='LOCALIZED', valid=self.reloc_valid)
        if 'gateway_status' not in self.stale:
            self.status('gateway_status', state=self.gw, reason=self.reason, real_output=not self.preview)
        if 'executor_status' not in self.stale:
            self.status('executor_status', state=self.exec_state, reason='')
        s.on_result(String(json.dumps(dict(confirmed=True, map='synthetic'))))
        if 'est' not in self.stale:
            e = EstimatorStatus()
            for k in ('attitude_status_flag', 'velocity_horiz_status_flag', 'velocity_vert_status_flag', 'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
                setattr(e, k, True)
            s.on_est(e)

    def tick(self):
        if self.ready_at is not None and self.clock.t >= self.ready_at:
            self.gw, self.reason, self.ready_at = 'READY', '', None
        c = self.client
        if c.finish_at is not None and self.clock.t >= c.finish_at:
            point = c.goals[-1][1].waypoints[0].pose
            self.position = self.r.T @ (np.array([point.position.x, point.position.y, point.position.z]) - self.translation)
            self.yaw = math.atan2(2 * point.orientation.w * point.orientation.z, 1 - 2 * point.orientation.z ** 2) - self.map_yaw
            c.state, c.result, c.finish_at = 3, NS(success=self.action_success, reason='all waypoints settled'), None
            self.gw, self.reason, self.exec_state = 'FAULT', nav.STOP_REASON, 'IDLE'
        self.publish()
        self.inject()

    def lookup_transform(self, *args):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.from_sec(self.clock.time() - (.8 if 'tf' in self.stale else 0))
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = self.translation
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(self.map_yaw / 2), math.cos(self.map_yaw / 2)
        return msg

    def check(self, path):
        self.checked.append(path)
        return NS(valid=self.check_valid)

    def prepare(self):
        self.prepare_calls.append(self.clock.t)
        self.gw, self.reason = 'PREPARED', ''
        self.ready_at = self.clock.t + (0.1 if self.preview else 1.05)
        self.publish()
        return NS(success=True, message='prepared')


class SquareTests(unittest.TestCase):
    def harness(self, **kwargs):
        h = Harness(**kwargs)
        self.addCleanup(h.close)
        return h

    def test_geometry_rotates_and_closes(self):
        p = np.array([.5, -.7, 1.2])
        for degrees in (0, 90, -90, 180, -179.9, 37):
            a = math.radians(degrees)
            f, right = np.array([math.cos(a), math.sin(a), 0]), np.array([math.sin(a), -math.cos(a), 0])
            goals = nav.square_points(p, a)
            np.testing.assert_allclose(np.diff(np.vstack([p, goals]), axis=0), [f, right, -f, -right], atol=1e-12)
            np.testing.assert_allclose(goals[-1], p)
            np.testing.assert_allclose(goals[:, 2], p[2])

    def test_nonfinite_geometry_rejected(self):
        for p, yaw in (([0, 0, float('nan')], 0), ([0, 0, 1], float('inf'))):
            with self.assertRaises(nav.Stopped):
                nav.square_points(p, yaw)

    def test_full_lap_hold_frozen_map_and_heading(self):
        h = self.harness()
        start, initial = h.clock.t, h.position.copy()
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 4)
        self.assertGreaterEqual(h.client.goals[0][0] - start, 3.)
        self.assertEqual(len(h.prepare_calls), 3)
        rec = h.runner.record
        np.testing.assert_allclose(rec['goals_odom'], nav.square_points(initial, .37))
        np.testing.assert_allclose(rec['goals_map'], np.array(rec['goals_odom']) @ h.r.T + h.translation)
        for i, (_, goal) in enumerate(h.client.goals):
            q = goal.waypoints[0].pose.orientation
            self.assertAlmostEqual(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z), .97)
            self.assertEqual(goal.waypoints[0].header.frame_id, 'drone_map')
            self.assertGreaterEqual(rec['legs'][i]['hold_elapsed_s'], 5.)
            if i:
                self.assertGreaterEqual(h.client.goals[i][0] - rec['legs'][i - 1]['hold_end_wall'] + 1000, 1.)
        self.assertEqual(rec['end'], 'SUCCEEDED')
        with open(h.runner.record_path) as f:
            self.assertEqual(json.load(f)['end'], 'SUCCEEDED')
        np.testing.assert_allclose(h.position, initial, atol=1e-12)

    def test_countdown_accepts_prepared_until_gateway_ready(self):
        h = self.harness()
        h.gw, h.ready_at = 'PREPARED', h.clock.t + .5
        h.publish()
        self.assertTrue(h.runner.run_lap())

    def test_bounds_reject_whole_square_before_first_goal(self):
        h = self.harness()
        h.position = np.array([2.9, 0., 1.])
        h.yaw = 0.
        h.publish()
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.client.goals, [])

    def test_occupied_target_never_sent(self):
        h = self.harness()
        h.check_valid = False
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.client.goals, [])

    def test_false_success_flag_never_advances(self):
        h = self.harness()
        h.action_success = False
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)
        self.assertNotIn('hold_start_wall', h.runner.record['legs'][0])
        self.assertEqual(h.prepare_calls, [])

    def test_midflight_takeover_is_latched_even_if_mode_returns(self):
        h = self.harness()
        def inject():
            if h.client.state == 1:
                h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL'))
                h.state.on_fcu(NS(connected=True, armed=True, mode='OFFBOARD'))
        h.inject = inject
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)
        self.assertEqual(h.client.cancelled, 1)
        self.assertEqual(h.prepare_calls, [])

    def during_hold(self, h, action):
        def inject():
            if h.runner.record and h.runner.record['legs'] and 'hold_start_wall' in h.runner.record['legs'][-1]:
                action()
        h.inject = inject

    def test_takeover_during_hold_stops_remainder(self):
        h = self.harness()
        self.during_hold(h, lambda: h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL')))
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)
        self.assertEqual(h.prepare_calls, [])
        self.assertLess(h.runner.record['legs'][0]['hold_elapsed_s'], 5.)

    def test_transient_input_fault_is_latched(self):
        h = self.harness()
        def fault():
            h.status('input_status', state='FAULT', reason='clock stale')
            h.status('input_status', state='READY', reason='')
        self.during_hold(h, fault)
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)

    def test_unexpected_gateway_fault_is_never_prepared_away(self):
        h = self.harness()
        self.during_hold(h, lambda: h.status('gateway_status', state='FAULT', reason='LIO/PX4 frame residual', real_output=True))
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.prepare_calls, [])

    def test_tf_reset_during_hold_stops(self):
        h = self.harness()
        self.during_hold(h, lambda: h.translation.__setitem__(0, h.translation[0] + .1))
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)

    def test_stale_inputs_and_tf_during_hold_stop(self):
        for name in ('odom', 'fcu', 'input_status', 'gateway_status', 'reloc', 'est', 'tf'):
            with self.subTest(name=name):
                h = Harness()
                try:
                    self.during_hold(h, lambda: h.stale.add(name))
                    self.assertFalse(h.runner.run_lap())
                    self.assertEqual(len(h.client.goals), 1)
                    self.assertEqual(h.prepare_calls, [])
                finally:
                    h.close()

    def test_ctrl_c_cancels_active_goal_and_records(self):
        h = self.harness()
        def interrupt():
            if h.client.state == 1:
                raise KeyboardInterrupt
        h.inject = interrupt
        with self.assertRaises(KeyboardInterrupt):
            h.runner.run_lap()
        self.assertEqual(h.client.cancelled, 1)
        self.assertEqual(h.runner.record['end'], 'INTERRUPTED')

    def test_ctrl_c_during_hold_records_partial_hold_without_next_goal(self):
        h = self.harness()
        def interrupt():
            raise KeyboardInterrupt
        self.during_hold(h, interrupt)
        with self.assertRaises(KeyboardInterrupt):
            h.runner.run_lap()
        self.assertEqual(len(h.client.goals), 1)
        self.assertLess(h.runner.record['legs'][0]['hold_elapsed_s'], 5.)
        self.assertEqual(h.runner.record['end'], 'INTERRUPTED')

    def test_preview_uses_four_goals_with_output_disabled(self):
        h = self.harness(preview=True)
        self.assertTrue(h.runner.preview())
        self.assertEqual(len(h.client.goals), 4)
        self.assertFalse(h.runner.record['real_output'])
        self.assertEqual(len(h.prepare_calls), 4)

    def test_preview_refuses_real_output(self):
        h = self.harness(preview=True)
        h.status('gateway_status', state='READY', reason='', real_output=True)
        with self.assertRaises(nav.Stopped):
            h.runner.preview()
        self.assertEqual(h.client.goals, [])
        self.assertEqual(h.prepare_calls, [])

    def test_starting_offboard_waits_for_new_edge_and_repeats_only_on_edge(self):
        h = self.harness()
        laps = []
        h.runner.run_lap = lambda: laps.append(h.clock.t)
        start = h.clock.t
        def edges():
            elapsed = h.clock.t - start
            if .4 < elapsed < .8 or 1.5 < elapsed < 1.9:
                h.offboard = False
            else:
                h.offboard = True
            h.publish()
        h.inject = edges
        with patch.object(rospy, 'is_shutdown', side_effect=lambda: h.clock.t - start > 2.8):
            h.runner.automatic()
        self.assertEqual(len(laps), 2)
        self.assertGreaterEqual(laps[0] - start, .8)
        self.assertGreaterEqual(laps[1] - start, 1.9)

    def test_countdown_exit_never_sends(self):
        h = self.harness()
        h.inject = lambda: h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL'))
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.client.goals, [])

    def test_edges_during_aborted_lap_are_consumed(self):
        h = self.harness()
        h.offboard = False
        h.publish()
        laps = []
        start = h.clock.t
        def lap():
            laps.append(h.clock.t)
            h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL'))
            h.state.on_fcu(NS(connected=True, armed=True, mode='OFFBOARD'))
            return False
        h.runner.run_lap = lap
        def enter():
            if h.clock.t - start > .2:
                h.offboard = True
                h.publish()
        h.inject = enter
        with patch.object(rospy, 'is_shutdown', side_effect=lambda: h.clock.t - start > 1.):
            h.runner.automatic()
        self.assertEqual(len(laps), 1)

    def test_new_lap_refreezes_current_pose(self):
        h = self.harness()
        self.assertTrue(h.runner.run_lap())
        first_path = h.runner.record_path
        h.position, h.yaw = np.array([-.6, .7, 1.3]), -.8
        h.gw, h.reason = 'READY', ''
        h.publish()
        self.assertTrue(h.runner.run_lap())
        np.testing.assert_allclose(h.runner.record['start_odom'], [-.6, .7, 1.3])
        np.testing.assert_allclose(h.runner.record['goals_odom'], nav.square_points([-.6, .7, 1.3], -.8))
        self.assertNotEqual(h.runner.record_path, first_path)

    def test_prepare_rejection_after_success_stops_next_leg(self):
        h = self.harness()
        def reject():
            h.prepare_calls.append(h.clock.t)
            return NS(success=False, message='alignment, inputs or exclusive ownership not ready')
        h.runner.prepare = reject
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)
        self.assertEqual(len(h.prepare_calls), 1)

    def test_failed_action_states_do_not_hold_or_advance(self):
        for terminal in (2, 4, 5, 8, 9):
            with self.subTest(terminal=terminal):
                h = Harness()
                try:
                    def reject():
                        if h.client.state == 1:
                            h.client.state, h.client.finish_at = terminal, None
                            h.client.result = NS(success=False, reason='rejected')
                    h.inject = reject
                    self.assertFalse(h.runner.run_lap())
                    self.assertEqual(len(h.client.goals), 1)
                    self.assertNotIn('hold_start_wall', h.runner.record['legs'][0])
                finally:
                    h.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
