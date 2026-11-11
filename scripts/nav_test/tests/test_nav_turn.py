#!/usr/bin/env python3
"""离线测试 test8（nav_turn）：把航点展开成"前进 / 原地转向"，机头朝前进方向。

复用 test_nav_global.GlobalHarness（假时钟 + 假 action + 假 TF + 遥测传送），不需要 ROS master、
不连飞控、不发布任何 MAVROS 设定值。

跑法（板端，先 source 三个 setup.bash）：
    cd /home/orangepi/mid360s_ws/scripts/nav_test/tests
    python3 -m unittest -v test_nav_turn
"""
import math
import os
import sys
import unittest
from unittest.mock import patch

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR) if os.path.basename(SCRIPT_DIR) == 'tests' else SCRIPT_DIR)
import nav_turn  # noqa: E402
import test_nav_global  # noqa: E402

ROUTE = test_nav_global.ROUTE        # 与 nav_global.DEFAULT_ROUTE 相同：(0,0,1) (2,0,1) (2,-2,1) (0,-2,1) (0,0,1)


class TurnHarness(test_nav_global.GlobalHarness):
    RUNNER = nav_turn.TurnRunner

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # nav_turn 自己 import 了 nav_forward.say，要单独接管才能看到它打的提示
        extra = patch.object(nav_turn, 'say', self.say_log.append)
        extra.start()
        self.patches.append(extra)


class TurnTests(unittest.TestCase):
    def harness(self, **kwargs):
        h = TurnHarness(**kwargs)
        self.addCleanup(h.close)
        return h

    def yaws_deg(self, h):
        return [round(test_nav_global.goal_yaw_deg(goal), 6) for _, goal in h.client.goals]

    def positions(self, h):
        return [[goal.waypoints[0].pose.position.x, goal.waypoints[0].pose.position.y,
                 goal.waypoints[0].pose.position.z] for _, goal in h.client.goals]

    def test_expands_square_into_move_turn_pairs(self):
        h = self.harness()
        h.place([.8, 0., 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        # 4 段前进 + 3 次 90° 原地转向
        self.assertEqual(h.runner.kinds, ['move', 'turn', 'move', 'turn', 'move', 'turn', 'move'])
        self.assertEqual(len(h.client.goals), 7)
        # 每一段飞行时的机头 = 该段的前进方向
        self.assertEqual([round(y, 3) for y in self.yaws_deg(h)], [0., -90., -90., 180., 180., 90., 90.])
        # 转向段的位置必须与上一段终点完全一致（任务层按 1 μm 判"重复同一终点"）
        pos = self.positions(h)
        for i, kind in enumerate(h.runner.kinds):
            if kind == 'turn':
                self.assertEqual(pos[i], pos[i - 1])
        # 记录里的 map 点是展开后的目标：前进到 P2、原地转、前进到 P3、原地转、…
        rec = h.runner.record
        np.testing.assert_allclose([p[:3] for p in rec['goals_map']],
                                   [[2., 0., 1.], [2., 0., 1.], [2., -2., 1.], [2., -2., 1.],
                                    [0., -2., 1.], [0., -2., 1.], [0., 0., 1.]], atol=1e-12)
        self.assertEqual(rec['end'], 'SUCCEEDED')

    def test_turn_legs_do_not_hold_and_move_legs_do(self):
        h = self.harness(hold=3.0)
        h.place([.8, 0., 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        holds = [round(leg['hold_elapsed_s'], 6) for leg in h.runner.record['legs']]
        self.assertEqual(len(holds), 7)
        for kind, hold in zip(h.runner.kinds, holds):
            self.assertAlmostEqual(hold, 3.0 if kind == 'move' else 0.0, delta=1e-6)

    def test_turn_is_never_skipped(self):
        # 位置几乎不动、航向差 90°：test7 的 skip_leg 会当成"已在点上"报错，test8 必须照常发出去
        h = self.harness()
        h.place([1.95, .02, 1.], yaw_deg=0.)     # 停在 P2 附近（离首点 1.95 m，仍允许开跑）
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(h.runner.kinds[:2], ['move', 'turn'])
        self.assertEqual([round(y, 3) for y in self.yaws_deg(h)[:2]], [0., -90.])

    def test_collinear_route_needs_no_turn(self):
        points = [(0., 0., 1., 0.), (1., 0., 1., 0.), (2., 0., 1., 0.)]
        h = self.harness(points=points, hold=0.0)
        h.place([0., 0., 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(h.runner.kinds, ['move', 'move'])
        self.assertEqual([round(y, 6) for y in self.yaws_deg(h)], [0., 0.])

    def test_reversal_turns_180(self):
        points = [(0., 0., 1., 0.), (2., 0., 1., 0.), (0., 0., 1., 0.)]
        h = self.harness(points=points, hold=0.0)
        h.place([0., 0., 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(h.runner.kinds, ['move', 'turn', 'move'])
        self.assertEqual([round(abs(y), 6) for y in self.yaws_deg(h)], [0., 180., 180.])

    def test_turn_rate_comes_from_nav_config(self):
        h = self.harness()
        h.place([.8, 0., 1.], yaw_deg=0.)
        h.cfg['yaw_rate_deg'] = 45.0
        self.assertTrue(h.runner.run_lap())
        self.assertTrue(any('yaw_rate_deg=45' in s for s in h.say_log), h.say_log)


if __name__ == '__main__':
    unittest.main()
