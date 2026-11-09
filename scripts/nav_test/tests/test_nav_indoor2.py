#!/usr/bin/env python3
"""Indoor 0.5 m fixed-heading square with independent 2 s holds."""
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import rospy
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_nav_square import Harness
from nav_sequence import SequenceRunner, sequence_points
from nav_square import SquareRunner, Stopped


class IndoorSquareTests(unittest.TestCase):
    def harness(self, **kwargs):
        h = Harness(**kwargs)
        self.addCleanup(h.close)
        cfg = yaml.safe_load((Path(__file__).resolve().parents[1]/'indoor_nav.yaml').read_text())['config']
        h.cfg.update(cfg)
        h.cfg['map_id'] = 'synthetic'
        h.args.test = 'test_indoor2'
        h.runner = SequenceRunner(h.args, h.cfg, h.state, h, h.prepare, h.check, h.client)
        return h

    def close(self, h):
        h.close()
        self._cleanups.pop()

    def test_geometry_fixed_heading_four_half_metre_sides(self):
        p = np.array([.1, -.2, .5])
        for degrees in (0, 90, -90, 180, -179.9, 37):
            yaw = math.radians(degrees)
            f = np.array([math.cos(yaw), math.sin(yaw), 0.])
            right = np.array([math.sin(yaw), -math.cos(yaw), 0.])
            goals, yaws, names = sequence_points('test_indoor2', p, yaw)
            np.testing.assert_allclose(np.diff(np.vstack([p, goals]), axis=0),
                                       .5*np.array([f,right,-f,-right]), atol=1e-12)
            np.testing.assert_allclose(goals[-1], p)
            np.testing.assert_allclose(goals[:,2], p[2])
            np.testing.assert_allclose(yaws, yaw)
            self.assertEqual(len(names), 4)
        with self.assertRaises(Stopped):
            sequence_points('test_indoor2', [0,0,float('nan')], 0.)

    def test_full_square_two_second_holds_map_heading_and_json(self):
        h = self.harness()
        h.position[2] = .5
        h.publish()
        initial, yaw, start = h.position.copy(), h.yaw, h.clock.t
        self.assertTrue(h.runner.run_lap())
        record = h.runner.record
        expected, _, _ = sequence_points('test_indoor2', initial, yaw)
        self.assertEqual(len(h.client.goals), 4)
        self.assertEqual(len(h.prepare_calls), 3)
        self.assertGreaterEqual(h.client.goals[0][0]-start, 3.)
        self.assertEqual(record['side_m'], .5)
        self.assertEqual(record['hold_s'], 2.)
        self.assertEqual(record['test'], 'test_indoor2')
        np.testing.assert_allclose(record['goals_odom'], expected)
        np.testing.assert_allclose(record['goals_map'], expected@h.r.T+h.translation)
        for index, leg in enumerate(record['legs']):
            self.assertGreaterEqual(leg['hold_elapsed_s'], 2.)
            self.assertLess(leg['hold_elapsed_s'], 2.1)
            q = h.client.goals[index][1].waypoints[0].pose.orientation
            self.assertAlmostEqual(math.atan2(2*q.w*q.z,1-2*q.z*q.z), yaw+h.map_yaw)
            self.assertEqual(len(h.checked[index].poses), 2)
        np.testing.assert_allclose(h.position, initial, atol=1e-12)
        self.assertTrue(Path(h.runner.record_path).name.startswith('goal_test_indoor2_'))
        self.assertEqual(json.loads(Path(h.runner.record_path).read_text())['end'], 'SUCCEEDED')

    def test_hold_settings_do_not_leak_to_other_tests(self):
        h = self.harness()
        self.assertEqual(h.runner.hold_seconds, 2.)
        for test in ('test4', 'test5', 'test_indoor1'):
            h.args.test = test
            runner = SequenceRunner(h.args, h.cfg, h.state, h, h.prepare, h.check, h.client)
            self.assertEqual(runner.hold_seconds, 5.)
            self.assertIsNone(runner.side_m)
        runner = SquareRunner(h.args, h.cfg, h.state, h, h.prepare, h.check, h.client)
        self.assertEqual((runner.side_m, runner.hold_seconds), (1., 5.))

    def test_later_goals_remain_frozen_after_position_and_yaw_drift(self):
        h = self.harness()
        initial, yaw, fired = h.position.copy(), h.yaw, [False]
        def drift():
            if h.runner.record and h.runner.record['legs'] and 'hold_start_wall' in h.runner.record['legs'][0] and not fired[0]:
                fired[0] = True
                h.position += [.04, -.03, .01]
                h.yaw += math.radians(2)
                h.publish()
        h.inject = drift
        self.assertTrue(h.runner.run_lap())
        np.testing.assert_allclose(h.runner.record['goals_odom'], sequence_points('test_indoor2', initial, yaw)[0])
        np.testing.assert_allclose(h.runner.record['goal_yaw_odom_deg'], math.degrees(yaw))

    def test_later_corner_out_of_margin_rejects_before_first_goal(self):
        h = self.harness()
        h.position, h.yaw = np.array([2., -2.3, .5]), 0.
        h.publish()
        self.assertFalse(h.runner.run_lap())
        self.assertFalse(h.client.goals)
        self.assertFalse(h.checked)

    def test_unknown_or_occupied_later_segment_stops_remainder(self):
        for reason in ('unknown footprint', 'occupied footprint'):
            h = self.harness()
            def check(path):
                response = h.check(path)
                return NS(valid=False, reason=reason) if len(h.checked)==2 else response
            h.runner.check = check
            self.assertFalse(h.runner.run_lap())
            self.assertEqual(len(h.client.goals), 1)
            self.assertEqual(len(h.checked), 2)
            self.close(h)

    def test_takeover_input_relocation_and_tf_faults_stop_any_leg_or_hold(self):
        for index in range(1,5):
            for phase in ('running', 'holding'):
                for fault in ('takeover', 'input', 'reloc', 'tf'):
                    with self.subTest(leg=index, phase=phase, fault=fault):
                        h = self.harness()
                        fired = [False]
                        def inject():
                            if fired[0] or len(h.client.goals)!=index:
                                return
                            if phase=='running' and h.client.state!=1:
                                return
                            if phase=='holding' and not (h.runner.record and 'hold_start_wall' in h.runner.record['legs'][-1]):
                                return
                            fired[0] = True
                            if fault=='takeover':
                                h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL'))
                                h.state.on_fcu(NS(connected=True, armed=True, mode='OFFBOARD'))
                            elif fault=='input':
                                h.status('input_status', state='FAULT', reason='clock stale')
                                h.status('input_status', state='READY', reason='')
                            elif fault=='reloc':
                                h.status('reloc', state='LOCALIZED', valid=False)
                                h.status('reloc', state='LOCALIZED', valid=True)
                            else:
                                h.translation[0] += .1
                        h.inject = inject
                        self.assertFalse(h.runner.run_lap())
                        self.assertTrue(fired[0])
                        self.assertEqual(len(h.client.goals), index)
                        self.assertEqual(h.client.cancelled, int(phase=='running'))
                        self.assertEqual(h.runner.record['end'], 'ABORTED')
                        self.close(h)

    def test_stream_loss_during_hold_stops_remainder(self):
        for name in ('odom', 'input_status', 'reloc', 'gateway_status', 'tf'):
            h = self.harness()
            def drop():
                if h.runner.record and h.runner.record['legs'] and 'hold_start_wall' in h.runner.record['legs'][0]:
                    h.stale.add(name)
            h.inject = drop
            self.assertFalse(h.runner.run_lap())
            self.assertEqual(len(h.client.goals), 1)
            self.close(h)

    def test_starting_offboard_and_countdown_takeover_send_nothing(self):
        h = self.harness()
        start = h.clock.t
        with patch.object(rospy, 'is_shutdown', side_effect=lambda: h.clock.t-start > .6):
            h.runner.automatic()
        self.assertFalse(h.client.goals)
        h.inject = lambda: h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL'))
        self.assertFalse(h.runner.run_lap())
        self.assertFalse(h.client.goals)

    def test_full_lap_repeats_only_on_fresh_mode_entry(self):
        h = self.harness()
        h.offboard = False
        h.publish()
        initial, start, stage = h.position.copy(), h.clock.t, [0, None]
        def edges():
            if stage[0]==0 and h.clock.t-start>.3:
                h.offboard, stage[0] = True, 1
            elif stage[0]==1 and h.runner.record and h.runner.record['end']=='SUCCEEDED':
                stage[0], stage[1] = 2, h.clock.t
            elif stage[0]==2 and h.clock.t-stage[1]>2.5:
                self.assertEqual(len(h.client.goals), 4)
                h.offboard, stage[0], stage[1] = False, 3, h.clock.t
            elif stage[0]==3 and h.clock.t-stage[1]>1.5:
                h.offboard, stage[0] = True, 4
            h.publish()
        h.inject = edges
        with patch.object(rospy, 'is_shutdown', side_effect=lambda: h.clock.t-start > 43.):
            h.runner.automatic()
        self.assertEqual(len(h.client.goals), 8)
        np.testing.assert_allclose(h.position, initial, atol=1e-12)

    def test_preview_four_goals_and_ctrl_c_cancel(self):
        h = self.harness(preview=True)
        self.assertTrue(h.runner.preview())
        self.assertEqual(len(h.client.goals), 4)
        self.assertFalse(h.runner.record['real_output'])
        self.close(h)
        h = self.harness()
        def interrupt():
            if h.client.state==1:
                raise KeyboardInterrupt
        h.inject = interrupt
        with self.assertRaises(KeyboardInterrupt):
            h.runner.run_lap()
        self.assertEqual(h.client.cancelled, 1)
        self.assertEqual(h.runner.record['end'], 'INTERRUPTED')

    def test_entrypoint_uses_indoor_defaults_and_separate_record_name(self):
        script=Path(__file__).resolve().parents[2]/'nav_test_indoor2.sh'
        probe=r'''
mkdir() { :; }
source() {
  builtin source "$@"
  nt_bringup() {
    python3 - "$TEST_NAME" "$MAP" "$RELOC_CONFIG" "$MAP_FRAME" "$NAV_CONFIG" "$PREVIEW" "$PREVIEW_RADIUS" "$PREVIEW_HALF_HEIGHT" <<'PY'
import json,sys
print(json.dumps(sys.argv[1:]))
PY
    rmdir "$RUN_DIR"
    exit 0
  }
}
builtin source "$0" "$@"
'''
        value=json.loads(subprocess.check_output(['bash','-c',probe,str(script),'--preview'],text=True))
        self.assertEqual(value[0], 'test_indoor2')
        self.assertTrue(value[1].endswith('/mapping/20261021_110311_indoor'))
        self.assertTrue(value[2].endswith('/config/indoor.yaml'))
        self.assertTrue(value[3].endswith('/config/map_frame.yaml'))
        self.assertTrue(value[4].endswith('/nav_test/indoor_nav.yaml'))
        self.assertEqual(value[5:], ['1','0.4','0.3'])


if __name__=='__main__':
    unittest.main(verbosity=2)
