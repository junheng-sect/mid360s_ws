#!/usr/bin/env python3
"""No master/FCU: actual ROS types, fake time and bounded flight services."""
import json
import math
import os
import sys
import unittest
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ExtendedState, EstimatorStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import nav_outdoor_full as nav
import nav_global
import nav_land
import test_nav_square as base


class FlightClient(base.Client):
    def send_goal(self, goal):
        super().send_goal(goal)
        self.finish_at = self.h.clock.t + .4
        self.arrive_at = self.h.clock.t + .15


class Harness(base.Harness):
    def __init__(self):
        super().__init__()
        self.patches.extend([patch.object(nav, 'time', self.clock),
                             patch.object(nav_global, 'time', self.clock),
                             patch.object(nav_land, 'time', self.clock),
                             patch.object(nav, 'say', side_effect=lambda _: None)])
        for p in self.patches[-4:]:
            p.start()
        self.position, self.translation = np.zeros(3), np.zeros(3)
        self.r = np.eye(3)
        self.yaw = self.map_yaw = 0.
        self.state = nav.FullState()
        self.fc = nav.Fc(self.state)
        self.state.fc = self.fc
        self.mode, self.armed, self.landed = 'POSCTL', False, 1
        self.const_pos = True
        self.ground_z = 0.
        self.takeoff_state = ''
        self.auto_arm = True
        self.auto_arm_at = None
        self.climb_at = None
        self.land_at = None
        self.land_state, self.land_calls, self.land_z0, self.land_z_at = '', 0, None, None
        self.climb_converges = True
        self.auto_disarm = True
        self.land_accepts = True
        self.never_touch = False
        self.takeoff_calls = self.prepare_takeoff_calls = 0
        self.mode_calls = []
        self.fcu_velocity = 0.
        self.freeze_action = False
        self.client = FlightClient(self)
        self.cfg.update(map_id='synthetic', takeoff_height=1., yaw_rate_deg=45.,
                        max_vel=1., max_acc=1., max_vel_z=.3, max_acc_z=.3, goal_boundary_margin=.30)
        self.args = NS(preview=False, delay=3., out=self.tmp.name)
        self.runner = nav.FullRunner(self.args, self.cfg, self.state, self, self.prepare, self.check,
                                     self.client, self.fc, self.prepare_takeoff, self.takeoff,
                                     self.land_start, self.set_mode)
        self.publish()
        self.publish()

    def publish(self):
        if not hasattr(self, 'mode'):
            return super().publish()
        s = self.state
        if 'fcu' not in self.stale:
            s.on_fcu(NS(connected=self.connected, armed=self.armed, mode=self.mode))
        if 'odom' not in self.stale:
            o = Odometry()
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.position
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
            s.on_odom(o)
        if 'fc_odom' not in self.stale:
            o = Odometry()
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.position
            self.fc.on_odom(o)
        if 'fc_vel' not in self.stale:
            v = TwistStamped()
            v.twist.linear.z = self.fcu_velocity
            self.fc.on_vel(v)
        if 'ext' not in self.stale:
            self.fc.on_ext(ExtendedState(landed_state=self.landed))
        if 'input_status' not in self.stale:
            self.status('input_status', state=self.input_state, reason='')
        if 'reloc' not in self.stale:
            self.status('reloc', state='LOCALIZED', valid=self.reloc_valid)
        if 'gateway_status' not in self.stale:
            self.status('gateway_status', state=self.gw, reason=self.reason, real_output=True,
                        takeoff=self.takeoff_state, land=self.land_state)
        self.status('executor_status', state=self.exec_state, reason='')
        s.on_result(String(json.dumps(dict(confirmed=True, map='synthetic'))))
        if 'est' not in self.stale:
            e = EstimatorStatus()
            for k in ('attitude_status_flag', 'velocity_horiz_status_flag', 'velocity_vert_status_flag',
                      'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
                setattr(e, k, True)
            e.const_pos_mode_status_flag = self.const_pos
            s.on_est(e)

    def tick(self):
        now = self.clock.t
        if self.state.phase == 'WAIT_ARM' and self.auto_arm and self.auto_arm_at is None:
            self.auto_arm_at = now + .15
        if self.auto_arm_at is not None and now >= self.auto_arm_at:
            self.armed, self.auto_arm_at = True, None
        if self.ready_at is not None and now >= self.ready_at:
            self.gw, self.reason, self.ready_at = 'READY', '', None
        if self.climb_at is not None and self.state.phase == 'TAKEOFF':
            dt = now - self.climb_at
            if dt > .10:
                self.landed, self.const_pos, self.position[2] = 2, False, .82
            if dt > .20:
                self.takeoff_state = 'DONE'
            if dt > .60 and self.climb_converges:
                self.position[2] = 1.
        c = self.client
        if c.state == 1 and not self.freeze_action:
            point = c.goals[-1][1].waypoints[0].pose
            goal = np.array([point.position.x, point.position.y, point.position.z])
            if now >= c.arrive_at:
                self.position = self.r.T @ (goal-self.translation)
                self.exec_state = 'HOLD'
            if now >= c.finish_at:
                self.yaw = math.atan2(2*point.orientation.w*point.orientation.z,
                                      1-2*point.orientation.z**2)-self.map_yaw
                c.state, c.result, c.finish_at = 3, NS(success=self.action_success, reason='settled'), None
                self.gw, self.reason, self.exec_state = 'FAULT', nav.STOP_REASON, 'IDLE'
        if self.land_state in ('DESCENDING', 'TOUCHED') and not self.never_touch:
            if self.land_state == 'DESCENDING':
                # PX4 tracks the gateway hold: 0.1 m/s down from the return point to the ground reference.
                self.position[2] = max(0., self.land_z0 - .1*(now - self.land_at))
                if self.position[2] <= 0.:
                    self.land_state, self.land_z_at = 'TOUCHED', now
                    self.landed, self.const_pos = 1, True
            elif self.auto_disarm and now - self.land_z_at > .35:
                self.armed = False
        self.publish()
        self.inject()

    def prepare_takeoff(self):
        self.prepare_takeoff_calls += 1
        self.gw, self.reason, self.takeoff_state = 'READY', '', 'GROUND'
        self.publish()
        return NS(success=True, message='prepared')

    def takeoff(self):
        self.takeoff_calls += 1
        self.climb_at, self.takeoff_state = self.clock.t, 'CLIMBING'
        return NS(success=True, message='climbing')

    def set_mode(self, base_mode, custom_mode):
        self.mode_calls.append((self.clock.t, custom_mode))
        if custom_mode == 'OFFBOARD':
            self.mode = 'OFFBOARD'
        self.publish()
        return NS(mode_sent=True)

    def land_start(self):
        self.land_calls += 1
        if not self.land_accepts:
            self.publish()
            return NS(success=False, message='not READY in OFFBOARD')
        self.land_at, self.land_z0 = self.clock.t, float(self.position[2])
        self.land_state, self.gw, self.reason = 'DESCENDING', 'READY', ''
        self.publish()
        return NS(success=True, message='descending')


class FullTests(unittest.TestCase):
    def harness(self):
        h = Harness()
        self.addCleanup(h.close)
        return h

    def test_route_and_profile(self):
        h = self.harness()
        nav.profile(h.cfg)
        pts = np.array([p[:3] for p in nav.ROUTE])
        d = np.diff(np.vstack([[0,0,1], pts]), axis=0)
        np.testing.assert_array_equal(d, [[2,0,0],[0,-2,0],[0,0,1],[-2,0,0],[0,0,-1],[0,2,0]])
        self.assertEqual(sum(np.linalg.norm(x[:2]) for x in d), 8.)
        self.assertEqual(nav.HOLDS, (0.,0.,1.,0.,1.,2.))
        with self.assertRaises(ValueError):
            nav.profile(dict(h.cfg, takeoff_height=1.5))

    def test_full_once_ground_constpos_height_gate_phase_records(self):
        h = self.harness()
        self.assertTrue(h.runner.run_once())
        self.assertEqual(len(h.client.goals), 6)
        self.assertEqual(h.prepare_takeoff_calls, 1)
        self.assertEqual(h.takeoff_calls, 1)
        self.assertEqual([x[1] for x in h.mode_calls], ['OFFBOARD'])
        self.assertEqual(h.land_calls, 1)
        self.assertGreaterEqual(h.mode_calls[0][0]-h.state.arm_at, 3.)
        self.assertGreaterEqual(h.client.goals[0][0]-h.climb_at if h.climb_at else
                                h.client.goals[0][0]+1000-h.runner.flight['takeoff_request_wall'], 2.6)
        self.assertEqual(h.runner.flight['end'], 'SUCCEEDED')
        self.assertEqual(h.runner.flight['touchdown_disarmed']['landed'], 1)
        self.assertFalse(h.runner.flight['touchdown_disarmed']['armed'])
        for i in (0,2):
            self.assertIn('TURN', [x['phase'] for x in h.runner.record['legs'][i]['phases']])
        self.assertIn('HOLD', [x['phase'] for x in h.runner.record['legs'][5]['phases']])
        with open(h.runner.flight_path) as f:
            self.assertEqual(json.load(f)['end'], 'SUCCEEDED')
        before = list(h.mode_calls)
        with self.assertRaises(nav.Stopped):
            h.runner.run_once()
        self.assertEqual(h.mode_calls, before)

    def test_startup_armed_or_wrong_ground_mode_rejected(self):
        for mode, armed in [('POSCTL',True), ('OFFBOARD',False), ('AUTO.LAND',False)]:
            with self.subTest(mode=mode, armed=armed):
                h = self.harness()
                h.mode, h.armed = mode, armed
                h.publish()
                self.assertFalse(h.runner.run_once())
                self.assertEqual(h.takeoff_calls, 0)
                self.assertEqual(h.mode_calls, [])

    def test_bad_origin_yaw_and_future_goal_bounds_rejected_before_arm(self):
        for position,yaw,translation in [([.31,0,0],0,[0,0,0]),([0,0,0],.18,[0,0,0]),
                                          ([0,0,0],0,[0,0,.2])]:
            with self.subTest(position=position,yaw=yaw):
                h = self.harness()
                h.position, h.yaw, h.translation = np.array(position,dtype=float), yaw, np.array(translation)
                h.publish()
                self.assertFalse(h.runner.run_once())
                self.assertEqual(h.prepare_takeoff_calls,0)
        h = self.harness()
        h.cfg['bounds_max']=[1.9,3,2.5]
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.prepare_takeoff_calls,0)

    def test_occupied_future_target_never_starts(self):
        h = self.harness()
        h.check_valid = False
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.mode_calls, [])

    def test_done_but_low_height_never_sends_route(self):
        h = self.harness()
        h.climb_converges=False
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.client.goals, [])
        self.assertEqual([x[1] for x in h.mode_calls], ['OFFBOARD'])

    def test_stable_window_resets_on_height_excursion(self):
        h = self.harness()
        resets=[]
        def inject():
            if h.climb_at and 1.5 < h.clock.t-h.climb_at < 1.7:
                h.position[2]=.85
                h.publish()
                resets.append(h.clock.t)
        h.inject=inject
        self.assertTrue(h.runner.run_once())
        self.assertGreaterEqual(h.client.goals[0][0]-max(resets), 2.)

    def test_countdown_disarm_cancelled_no_offboard(self):
        h = self.harness()
        def inject():
            if h.state.phase=='COUNTDOWN':
                h.state.on_fcu(NS(connected=True,armed=False,mode='POSCTL'))
                h.state.on_fcu(NS(connected=True,armed=True,mode='POSCTL'))
        h.inject=inject
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.mode_calls,[])

    def test_takeover_and_input_loss_are_latched_on_recovery(self):
        for kind in ('mode','input','map'):
            with self.subTest(kind=kind):
                h=self.harness()
                def inject():
                    if h.client.state==1:
                        if kind=='mode':
                            h.state.on_fcu(NS(connected=True,armed=True,mode='POSCTL'))
                            h.state.on_fcu(NS(connected=True,armed=True,mode='OFFBOARD'))
                        elif kind=='input':
                            h.status('input_status',state='FAULT',reason='brief fault')
                            h.status('input_status',state='READY',reason='')
                        else:
                            h.state.on_result(String(json.dumps(dict(confirmed=True,map='wrong'))))
                            h.state.on_result(String(json.dumps(dict(confirmed=True,map='synthetic'))))
                h.inject=inject
                self.assertFalse(h.runner.run_once())
                self.assertEqual(len(h.client.goals),1)
                self.assertEqual(h.client.cancelled,1)
                self.assertEqual(h.land_calls, 0)

    def test_map_reset_during_takeoff_detected_not_recaptured(self):
        h=self.harness()
        def inject():
            if h.climb_at and h.clock.t-h.climb_at>.2:
                h.translation[0]=.02
        h.inject=inject
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.client.goals,[])
        self.assertEqual(h.runner.reference[1][0],0.)

    def test_leg_deadline_does_not_wait_125_seconds(self):
        h=self.harness()
        h.freeze_action=True
        start=h.clock.t
        self.assertFalse(h.runner.run_once())
        self.assertLess(h.clock.t-start,30)
        self.assertEqual(h.client.cancelled,1)

    def test_false_route_result_never_lands(self):
        h=self.harness()
        h.action_success=False
        self.assertFalse(h.runner.run_once())
        self.assertEqual(len(h.client.goals),1)
        self.assertEqual(h.land_calls, 0)

    def test_land_failure_and_no_disarm_timeout(self):
        for kind in ('service', 'disarm', 'touch'):
            with self.subTest(kind=kind):
                h = self.harness()
                h.land_accepts = kind != 'service'
                h.auto_disarm = kind != 'disarm'
                h.never_touch = kind == 'touch'
                self.assertFalse(h.runner.run_once())
                self.assertEqual(len(h.client.goals), 6)
                self.assertEqual(h.runner.flight['end'], 'ABORTED')
                self.assertEqual([x[1] for x in h.mode_calls], ['OFFBOARD'])
                self.assertEqual(h.land_calls, 1)
                if kind != 'service':
                    self.assertIn('超时', h.runner.flight['reason'])

    def test_land_takeover_latched_does_not_reassert_modes(self):
        h = self.harness()
        def inject():
            if h.land_state == 'DESCENDING':
                h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL'))
                h.state.on_fcu(NS(connected=True, armed=True, mode='OFFBOARD'))
        h.inject = inject
        self.assertFalse(h.runner.run_once())
        self.assertEqual([x[1] for x in h.mode_calls], ['OFFBOARD'])
        self.assertEqual(h.land_calls, 1)

    def test_const_pos_before_the_landed_state_does_not_abort(self):
        h = self.harness()
        seen = {'done': False}
        def inject():
            # PX4 raises const_pos a moment before the landed state flips at touchdown.
            if h.land_state == 'DESCENDING' and h.position[2] <= .03 and not seen['done']:
                seen['done'] = True
                h.const_pos = True
                h.publish()
        h.inject = inject
        self.assertTrue(h.runner.run_once())
        self.assertTrue(seen['done'])

    def test_extended_and_local_telemetry_stale_aborts(self):
        for key in ('ext','fc_odom','fc_vel','fcu','odom','est','gateway_status'):
            with self.subTest(key=key):
                h=self.harness()
                h.inject=lambda: h.stale.add(key) if h.climb_at else None
                self.assertFalse(h.runner.run_once())
                self.assertEqual(h.client.goals,[])

    def test_airborne_disarm_is_not_complete_success(self):
        h = self.harness()
        h.never_touch = True
        h.inject = lambda: setattr(h, 'armed', False) if h.land_state == 'DESCENDING' else None
        self.assertFalse(h.runner.run_once())
        self.assertNotIn('touchdown_disarmed', h.runner.flight)

    def test_nonidentity_map_keeps_absolute_goals(self):
        h=self.harness()
        h.map_yaw=.4
        h.r=np.array([[math.cos(.4),-math.sin(.4),0],[math.sin(.4),math.cos(.4),0],[0,0,1]])
        h.yaw=-.4
        h.publish()
        self.assertTrue(h.runner.run_once())
        np.testing.assert_allclose(h.runner.record['goals_map'],np.array(nav.ROUTE)[:,:3])

    def test_return_drift_never_triggers_land(self):
        h=self.harness()
        def inject():
            if len(h.client.goals)==6 and h.client.state==3:
                h.position[0]=.2
                h.publish()
        h.inject=inject
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.land_calls, 0)

    def test_arbitrary_gateway_fault_never_cleared(self):
        h=self.harness()
        def inject():
            if h.client.state==1:
                h.status('gateway_status',state='FAULT',reason='alignment residual',real_output=True)
                h.status('gateway_status',state='READY',reason='',real_output=True)
        h.inject=inject
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.prepare_calls,[])
        self.assertEqual(h.land_calls, 0)

    def test_stalled_service_aborts_without_followup_or_retry(self):
        for name in ('check_trajectory', 'takeoff', 'set_mode', 'land'):
            with self.subTest(service=name):
                h = self.harness()
                release, calls = threading.Event(), []
                self.addCleanup(release.set)
                def stalled(*args, **kw):
                    calls.append(kw)
                    release.wait(5)
                    return NS(success=True, mode_sent=True, valid=True)
                if name == 'check_trajectory':
                    h.runner.check = lambda path: h.runner.service_call(name, stalled, path)
                elif name == 'takeoff':
                    h.runner.takeoff = lambda: h.runner.service_call(name, stalled)
                elif name == 'land':
                    h.runner.land_start = lambda: h.runner.service_call(name, stalled)
                else:
                    h.runner.set_mode = lambda **kw: h.runner.service_call(name, stalled, **kw)
                with patch.object(nav, 'SERVICE_TIMEOUT', .10):
                    self.assertFalse(h.runner.run_once())
                self.assertEqual(len(calls), 1)
                self.assertEqual(len(h.client.goals), 6 if name == 'land' else 0)
                self.assertIn('服务调用超时', h.runner.flight['reason'])
                with self.assertRaises(nav.Stopped):
                    h.runner.run_once()
                release.set()

    def test_offboard_entry_recovery_is_takeover_not_retry(self):
        h = self.harness()
        def mode_call(**kw):
            h.mode_calls.append((h.clock.t, kw['custom_mode']))
            for mode in ('OFFBOARD', 'POSCTL', 'OFFBOARD'):
                h.state.on_fcu(NS(connected=True, armed=True, mode=mode))
            return NS(mode_sent=True)
        h.runner.set_mode = lambda **kw: h.runner.service_call('set_mode', mode_call, **kw)
        self.assertFalse(h.runner.run_once())
        self.assertEqual(h.takeoff_calls, 0)
        self.assertEqual(len(h.mode_calls), 1)

    def test_invalid_telemetry_still_writes_abort_record(self):
        h = self.harness()
        def inject():
            if h.climb_at:
                h.position[0] = float('nan')
                h.publish()
        h.inject = inject
        self.assertFalse(h.runner.run_once())
        with open(h.runner.flight_path) as f:
            record = json.load(f)
        self.assertEqual(record['end'], 'ABORTED')
        self.assertIsNone(record['stop_state']['odom_position'])

    def test_cancel_failure_preserves_abort_record(self):
        h = self.harness()
        h.freeze_action = True
        h.client.cancel_goal = lambda: (_ for _ in ()).throw(RuntimeError('mock cancel transport'))
        self.assertFalse(h.runner.run_once())
        self.assertIn('取消请求异常', h.runner.flight['reason'])
        self.assertEqual(h.runner.flight['end'], 'ABORTED')

    def test_preview_never_constructs_flight_services(self):
        h = self.harness()
        args = NS(preview=True, delay=3., out=h.tmp.name)
        with patch.object(nav, 'arguments', return_value=args), \
                patch.object(rospy, 'ServiceProxy', side_effect=AssertionError('flight service constructed')), \
                patch.object(nav_global, 'serve', return_value=0) as serve:
            self.assertEqual(nav.main(), 0)
        self.assertEqual(serve.call_args[0][0], 'nav_test10_preview')
        runner_cls = serve.call_args[0][2]
        with self.assertRaises(ValueError):
            runner_cls(args, dict(h.cfg, yaw_rate_deg=30.), h.state, h, h.prepare, h.check, h.client)

    def test_main_loads_the_config_and_constructs_the_flight_runner(self):
        """Exercises the real (non-preview) main() body: no name may be missing there."""
        h = self.harness()
        cfg = dict(h.cfg)
        state_calls = {'est': 0}
        def est_problem():
            # The estimator stream comes up late on cold MAVROS/FCU starts (11-12 test10): stale, then fine.
            state_calls['est'] += 1
            return '飞控 /mavros/estimator_status 2 s 没更新' if state_calls['est'] <= 2 else ''
        state = NS(fcu=NS(armed=False), initial_armed=False,
                   ekf_problem=est_problem, resting_only=lambda: False)
        client = Mock(); client.wait_for_server.return_value = True
        runner = Mock(); runner.inputs_problem.return_value = ''
        runner.run_once.return_value = True
        fc = Mock(); fc.ext_fresh.return_value = True
        fc.pos.return_value, fc.descent.return_value = np.zeros(3), 0.
        args = NS(preview=False, delay=3., out=h.tmp.name, nav_config='')
        wanted = []
        with patch.object(nav, 'say') as say_mock, \
                patch.object(nav, 'arguments', return_value=args), \
                patch.object(rospy, 'wait_for_service',
                             side_effect=lambda name, timeout=None: wanted.append(name)), \
                patch.object(rospy, 'has_param', return_value=True), \
                patch.object(rospy, 'get_param', return_value=cfg), \
                patch.object(rospy, 'Duration'), patch.object(rospy, 'ServiceProxy'), \
                patch.object(nav.actionlib, 'SimpleActionClient', return_value=client), \
                patch.object(nav, 'FullState', return_value=state), \
                patch.object(nav, 'Fc', return_value=fc), \
                patch.object(nav.tf2_ros, 'Buffer'), patch.object(nav.tf2_ros, 'TransformListener'), \
                patch.object(nav, 'FullRunner', return_value=runner) as ctor:
            self.assertEqual(nav.main(), 0)
        self.assertIn('/drone_nav/land', wanted)
        self.assertEqual(len(wanted), 6)
        self.assertGreaterEqual(state_calls['est'], 42)  # waited the 2 s stability window
        self.assertTrue(any('estimator_status' in str(c.args[0]) for c in say_mock.call_args_list))


if __name__=='__main__':
    unittest.main()
