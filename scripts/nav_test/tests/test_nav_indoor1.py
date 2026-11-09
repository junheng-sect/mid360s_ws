#!/usr/bin/env python3
"""Indoor single-leg semantics using the existing fake clock/action harness."""
import json
import math
from pathlib import Path
import sys
import subprocess
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import rospy
import yaml
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_nav_square import Harness
from nav_sequence import SequenceRunner, sequence_points
from nav_square import Stopped


class IndoorTests(unittest.TestCase):
    def harness(self, **kwargs):
        h = Harness(**kwargs)
        self.addCleanup(h.close)
        h.cfg['goal_boundary_margin'] = .3
        h.args.test = 'test_indoor1'
        h.runner = SequenceRunner(h.args, h.cfg, h.state, h, h.prepare, h.check, h.client)
        return h

    def test_forward_geometry_heading_height(self):
        p = np.array([.1, -.2, 1.2])
        for degrees in (0, 90, -90, 180, -179.9, 37):
            a = math.radians(degrees)
            goals, yaws, names = sequence_points('test_indoor1', p, a)
            np.testing.assert_allclose(goals[0]-p, [math.cos(a), math.sin(a), 0.], atol=1e-12)
            np.testing.assert_allclose(yaws, [a])
            self.assertEqual(names, ('前进 1 m',))
        for p, a in (([0, 0, float('nan')], 0), ([0, 0, 1], float('inf'))):
            with self.assertRaises(Stopped):
                sequence_points('test_indoor1', p, a)

    def test_freezes_after_countdown_and_checks_whole_segment(self):
        h = self.harness()
        initial = h.clock.t
        def drift_during_delay():
            if h.clock.t-initial < 2.9:
                h.position, h.yaw = np.array([-.3, .4, 1.2]), -.5
                h.publish()
        h.inject = drift_during_delay
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)
        self.assertGreaterEqual(h.client.goals[0][0]-initial, 3.)
        rec = h.runner.record
        self.assertEqual(rec['test'], 'test_indoor1')
        np.testing.assert_allclose(rec['start_odom'], [-.3, .4, 1.2])
        expected = np.array([-.3+math.cos(-.5), .4+math.sin(-.5), 1.2])
        np.testing.assert_allclose(rec['goals_odom'], [expected])
        np.testing.assert_allclose(rec['goals_map'], [h.r@expected+h.translation])
        np.testing.assert_allclose(rec['goal_yaw_odom_deg'], [math.degrees(-.5)])
        self.assertEqual(len(h.checked), 1)
        self.assertEqual(len(h.checked[0].poses), 2)
        self.assertGreaterEqual(rec['legs'][0]['hold_elapsed_s'], 5.)
        with open(h.runner.record_path) as f:
            self.assertEqual(json.load(f)['end'], 'SUCCEEDED')

    def test_two_fresh_entries_always_forward_then_hover(self):
        h = self.harness()
        h.offboard = False
        h.publish()
        initial, start = h.position.copy(), h.clock.t
        forward = np.array([math.cos(h.yaw), math.sin(h.yaw), 0.])
        stage = [0, None]
        def edges():
            if stage[0] == 0 and h.clock.t-start > .3:
                h.offboard, stage[0] = True, 1
            elif stage[0] == 1 and h.runner.record and h.runner.record['end'] == 'SUCCEEDED':
                h.offboard, stage[0], stage[1] = False, 2, h.clock.t
            elif stage[0] == 2 and h.clock.t-stage[1] > 1.5:
                h.offboard, stage[0] = True, 3
            h.publish()
        h.inject = edges
        with patch.object(rospy, 'is_shutdown', side_effect=lambda: h.clock.t-start > 25.):
            h.runner.automatic()
        self.assertEqual(len(h.client.goals), 2)
        np.testing.assert_allclose(h.position, initial+2*forward, atol=1e-12)
        files = list(Path(h.tmp.name).glob('goal_test_indoor1_*.json'))
        self.assertEqual(len(files), 2)
        self.assertTrue(all(json.loads(p.read_text())['end'] == 'SUCCEEDED' for p in files))

    def test_starting_offboard_never_sends(self):
        h = self.harness()
        start = h.clock.t
        with patch.object(rospy, 'is_shutdown', side_effect=lambda: h.clock.t-start > 4.):
            h.runner.automatic()
        self.assertFalse(h.client.goals)

    def test_countdown_takeover_latched_even_if_mode_returns(self):
        h = self.harness()
        def takeover():
            h.state.on_fcu(NS(connected=True, armed=True, mode='POSCTL'))
            h.state.on_fcu(NS(connected=True, armed=True, mode='OFFBOARD'))
        h.inject = takeover
        self.assertFalse(h.runner.run_lap())
        self.assertFalse(h.client.goals)

    def test_boundary_margin_blocks_later_forward_step(self):
        h = self.harness()
        h.position, h.yaw = np.array([1., 0., 1.]), 0.
        h.publish()
        self.assertTrue(h.runner.run_lap())
        h.gw, h.reason = 'READY', ''
        h.publish()
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)

    def test_path_rejected_without_action(self):
        h = self.harness()
        h.check_valid = False
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.checked[0].poses), 2)
        self.assertFalse(h.client.goals)

    def test_wrong_map_never_sends(self):
        h = self.harness()
        h.state.result['map'] = 'other-map'
        self.assertFalse(h.runner.run_lap())
        self.assertFalse(h.client.goals)

    def test_stale_inputs_and_tf_never_send(self):
        for name in ('odom', 'input_status', 'reloc', 'tf'):
            with self.subTest(name=name):
                h = self.harness()
                h.stale.add(name)
                self.assertFalse(h.runner.run_lap())
                self.assertFalse(h.client.goals)
                h.close()
                self._cleanups.pop()

    def test_transient_fault_and_tf_reset_cancel_active_leg(self):
        for fault in ('input', 'tf', 'map'):
            with self.subTest(fault=fault):
                h = self.harness()
                def inject():
                    if h.client.state != 1:
                        return
                    if fault == 'input':
                        h.status('input_status', state='FAULT', reason='clock stale')
                        h.status('input_status', state='READY', reason='')
                    elif fault == 'tf':
                        h.translation[0] += .1
                    else:
                        h.state.on_result(String(json.dumps(dict(confirmed=True, map='other-map'))))
                h.inject = inject
                self.assertFalse(h.runner.run_lap())
                self.assertEqual(len(h.client.goals), 1)
                self.assertEqual(h.client.cancelled, 1)
                self.assertEqual(h.runner.record['end'], 'ABORTED')
                h.close()
                self._cleanups.pop()

    def test_preview_single_leg_and_real_output_rejected(self):
        h = self.harness(preview=True)
        self.assertTrue(h.runner.preview())
        self.assertEqual(len(h.client.goals), 1)
        self.assertFalse(h.runner.record['real_output'])
        self.assertEqual(len(h.checked[0].poses), 2)
        h.status('gateway_status', state='READY', reason='', real_output=True)
        with self.assertRaises(Stopped):
            h.runner.preview()
        self.assertEqual(len(h.client.goals), 1)

    def test_indoor_config_keeps_measured_geometry_and_rejects_unknown(self):
        base = Path(__file__).resolve().parents[1]
        indoor = yaml.safe_load((base/'indoor_nav.yaml').read_text())['config']
        outdoor = yaml.safe_load((base/'outdoor_nav.yaml').read_text())['config']
        for key in ('lever_body', 'sensor_offset_imu', 'body_half_extent', 'clearance',
                    'bounds_max', 'goal_boundary_margin'):
            self.assertEqual(indoor[key], outdoor[key])
        self.assertEqual(indoor['bounds_min'][:2], outdoor['bounds_min'][:2])
        self.assertFalse(indoor['allow_unknown_above'])
        self.assertEqual((indoor['max_vel'], indoor['max_acc']), (.5, .5))
        self.assertEqual(indoor['map_id'], '/home/orangepi/test_data/mapping/20261021_110311_indoor')

    def test_low_height_admission_keeps_boundary_margin(self):
        base = Path(__file__).resolve().parents[1]
        cfg = yaml.safe_load((base/'indoor_nav.yaml').read_text())['config']
        for z, accepted in ((.298124945, False), (.399, False), (.4, True), (.568023947, True)):
            with self.subTest(z=z):
                h = self.harness()
                h.cfg.update(cfg)
                h.cfg['map_id'] = 'synthetic'
                h.runner = SequenceRunner(h.args, h.cfg, h.state, h, h.prepare, h.check, h.client)
                h.position, h.yaw = np.array([0., 0., z]), 0.
                h.publish()
                self.assertEqual(h.runner.run_lap(), accepted)
                self.assertEqual(len(h.client.goals), int(accepted))
                self.assertEqual(h.cfg['goal_boundary_margin'], .3)
                h.close()
                self._cleanups.pop()

    def test_entrypoint_defaults_and_cli_override_without_bringup(self):
        # Exercise the real shell argument/default handling. Stop before ROS,
        # avoid creating a flight-record directory, and remove the empty temp.
        scripts = Path(__file__).resolve().parents[2]
        probe = r'''
mkdir() { :; }
source() {
  builtin source "$@"
  nt_bringup() {
    python3 - "$MAP" "$RELOC_CONFIG" "$MAP_FRAME" "$NAV_CONFIG" "$PREVIEW" "$PREVIEW_RADIUS" "$PREVIEW_HALF_HEIGHT" <<'PY'
import json, sys
print(json.dumps(sys.argv[1:]))
PY
    rmdir "$RUN_DIR"
    exit 0
  }
}
builtin source "$0" "$@"
'''
        def run(script, *args):
            return json.loads(subprocess.check_output(['bash', '-c', probe, str(scripts/script), *args], text=True))
        indoor = run('nav_test_indoor1.sh', '--preview')
        self.assertTrue(indoor[0].endswith('/mapping/20261021_110311_indoor'))
        self.assertTrue(indoor[1].endswith('/config/indoor.yaml'))
        self.assertTrue(indoor[2].endswith('/config/map_frame.yaml'))
        self.assertTrue(indoor[3].endswith('/nav_test/indoor_nav.yaml'))
        self.assertEqual(indoor[4:], ['1', '0.4', '0.3'])
        overridden = run('nav_test_indoor1.sh', '--map', '/tmp', '--reloc-config', 'custom-reloc',
                         '--map-frame', 'custom-frame', '--nav-config', 'custom-nav')
        self.assertEqual(overridden[:5], ['/tmp', 'custom-reloc', 'custom-frame', 'custom-nav', '0'])
        for script in ('nav_test1.sh', 'nav_test2.sh', 'nav_test4.sh'):
            outdoor = run(script, '--preview')
            self.assertTrue(outdoor[0].endswith('/mapping/20261105_114605_outdoor'))
            self.assertTrue(outdoor[1].endswith('/config/outdoor.yaml'))
            self.assertTrue(outdoor[2].endswith('/config/map_frame_outdoor.yaml'))
            self.assertEqual(outdoor[4:], ['1', '0.2', '0.15'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
