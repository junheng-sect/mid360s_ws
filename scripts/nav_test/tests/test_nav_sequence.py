#!/usr/bin/env python3
"""Test new routes with the existing deterministic fake-action harness."""
import math
import os
import sys
import unittest
from unittest.mock import patch
import numpy as np
import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_nav_square import Harness
from nav_sequence import SequenceRunner, sequence_points
from nav_square import Stopped
from drone_navigation.core import check_goal_bounds, wrap


class SequenceTests(unittest.TestCase):
    def harness(self, test='test5'):
        h = Harness()
        self.addCleanup(h.close)
        h.cfg.update(bounds_min=[-5., -5., .4], bounds_max=[5., 5., 2.5], goal_boundary_margin=.3)
        h.args.test = test
        h.runner = SequenceRunner(h.args, h.cfg, h.state, h, h.prepare, h.check, h.client)
        return h

    def test_geometry_and_yaw_wrap(self):
        p = np.array([.1, -.2, 1.2])
        for yaw in (0, math.pi/2, -math.pi/2, math.radians(179)):
            f = np.array([math.cos(yaw), math.sin(yaw), 0.])
            q, a, _ = sequence_points('test4', p, yaw)
            np.testing.assert_allclose(q, [p+2*f, p, p+4*f, p])
            np.testing.assert_allclose(a, yaw)
            q, a, _ = sequence_points('test5', p, yaw)
            np.testing.assert_allclose(q[:4], [p+[0,0,.5], p, p+f+[0,0,.5], p])
            np.testing.assert_allclose(q[4:], np.tile(p, (4,1)))
            np.testing.assert_allclose([wrap(x) for x in np.diff(a[3:])], math.pi/2)
        with self.assertRaises(Stopped): sequence_points('test5', [0,0,float('nan')], 0)

    def test_both_full_sequences(self):
        for test, count in (('test4', 4), ('test5', 8)):
            with self.subTest(test=test):
                h = self.harness(test)
                p = h.position.copy()
                self.assertTrue(h.runner.run_lap())
                self.assertEqual(len(h.client.goals), count)
                self.assertEqual(h.runner.record['test'], test)
                self.assertEqual(len(h.prepare_calls), count-1)
                for leg in h.runner.record['legs']:
                    self.assertGreaterEqual(leg['hold_elapsed_s'], 5.)
                    self.assertIn('end_height_error_m', leg)
                np.testing.assert_allclose(h.position, p, atol=1e-12)
                for path in h.checked:
                    self.assertEqual(len(path.poses), 2 if test=='test5' else 1)
                h.close()
                self._cleanups.pop()  # harness already closed before next subtest

    def test_long_route_or_height_margin_rejects_all(self):
        for test, p in (('test4', [1.,0,1.]), ('test5', [0.,0,1.8])):
            h = self.harness(test)
            h.position, h.yaw = np.array(p), 0.
            h.publish()
            self.assertFalse(h.runner.run_lap())
            self.assertFalse(h.client.goals)
            h.close(); self._cleanups.pop()

    def test_unknown_climb_rejected_before_action(self):
        h = self.harness()
        h.check_valid = False
        self.assertFalse(h.runner.run_lap())
        self.assertFalse(h.client.goals)

    def test_takeover_during_height_does_not_reach_yaw(self):
        h = self.harness()
        def takeover():
            if h.client.goals:
                h.offboard = False
                h.publish()
        h.inject = takeover
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 1)
        self.assertGreaterEqual(h.client.cancelled, 1)

    def test_starting_offboard_does_not_trigger(self):
        h = self.harness()
        t = h.clock.t
        with patch.object(rospy, 'is_shutdown', side_effect=lambda: h.clock.t-t > .5):
            h.runner.automatic()
        self.assertFalse(h.client.goals)

    def test_boundary_default_invalid_and_tolerance(self):
        c = dict(bounds_min=[-3,-3,.4], bounds_max=[3,3,2.5])
        self.assertTrue(check_goal_bounds([[3,0,1]], c))
        c['goal_boundary_margin'] = .3
        self.assertTrue(check_goal_bounds([[2.7,0,1]], c))
        for p in ([2.71,0,1], [0,0,.69], [0,0,2.21]):
            with self.assertRaisesRegex(ValueError, 'clearance'):
                check_goal_bounds([p], c)
        for m in (-1, float('nan'), 2):
            c['goal_boundary_margin'] = m
            with self.assertRaises(ValueError): check_goal_bounds([], c)


if __name__ == '__main__': unittest.main(verbosity=2)
