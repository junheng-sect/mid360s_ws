#!/usr/bin/env python3
"""离线测试 test7（nav_global）：drone_map 绝对航点、每点保持、已在点上跳过、反变换与拒绝条件。

复用 test_nav_square.Harness 的假时钟/假 action/假 TF/遥测传送：不需要 ROS master，不连飞控，
不发布任何 MAVROS 设定值。

跑法（板端，先 source 三个 setup.bash，见 nav_test/README.md）：
    cd /home/orangepi/mid360s_ws/scripts/nav_test/tests
    python3 -m unittest -v test_nav_global
"""
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR) if os.path.basename(SCRIPT_DIR) == 'tests' else SCRIPT_DIR)
import nav_global  # noqa: E402
import test_nav_square  # noqa: E402

ROUTE = nav_global.DEFAULT_ROUTE


class GlobalHarness(test_nav_square.Harness):
    """换上 GlobalRunner，并把假的地图变换改小，让首点就在假飞机附近。"""

    RUNNER = nav_global.GlobalRunner

    def __init__(self, points=ROUTE, hold=3.0, max_start_offset=2.0, check_segments=False, **kwargs):
        self.points, self.hold = points, hold
        self.max_start_offset, self.check_segments = max_start_offset, check_segments
        super().__init__(**kwargs)
        # 一份不大但非零的 drone_map<-drone_odom 变换：既验证反变换，又让首点在附近
        self.map_yaw, self.translation = .35, np.array([.4, -.2, .05])
        self.r = np.array([[math.cos(self.map_yaw), -math.sin(self.map_yaw), 0.],
                           [math.sin(self.map_yaw), math.cos(self.map_yaw), 0.], [0., 0., 1.]])
        # nav_global 自己用的 time/say 也要换成本测试的假时钟与静音
        self.say_log = []
        extra = [patch.object(nav_global, 'time', self.clock),
                 patch.object(nav_global, 'say', self.say_log.append)]
        for p in extra:
            p.start()
        self.cfg['goal_boundary_margin'] = .30     # 与 outdoor_nav.yaml 一致，低高度提示才有意义
        self.patches.extend(extra)

    def make_args(self):
        return NS(preview=self.preview, delay=3.0, out=self.tmp.name, points=list(self.points),
                  hold=self.hold, max_start_offset=self.max_start_offset, check_segments=self.check_segments)

    def place(self, map_xyz, yaw_deg=0.0):
        """把假飞机放到 drone_map 的给定位置/航向（换回 odom 再发布）。"""
        self.position = self.r.T @ (np.array(map_xyz, float) - self.translation)
        self.yaw = math.radians(yaw_deg) - self.map_yaw
        self.publish()


def goal_yaw_deg(goal):
    q = goal.waypoints[0].pose.orientation
    return math.degrees(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))


class GlobalTests(unittest.TestCase):
    def harness(self, **kwargs):
        h = GlobalHarness(**kwargs)
        self.addCleanup(h.close)
        return h

    def test_sends_every_absolute_point_untouched(self):
        h = self.harness()
        h.place([.8, 0., 1.], yaw_deg=0.)      # 离首点 0.8 m：不跳过
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), len(ROUTE))
        for (_, goal), point in zip(h.client.goals, ROUTE):
            self.assertEqual(goal.waypoints[0].header.frame_id, 'drone_map')
            self.assertEqual(goal.map_id, 'synthetic')
            pos = goal.waypoints[0].pose.position
            # 数值必须与配置的绝对点完全一致：不受起点位置影响
            np.testing.assert_allclose([pos.x, pos.y, pos.z], point[:3], atol=1e-12)
            self.assertAlmostEqual(goal_yaw_deg(goal), point[3], places=9)
        self.assertEqual(len(h.prepare_calls), len(ROUTE) - 1)

    def test_odom_goals_round_trip_and_hold(self):
        h = self.harness()
        h.place([.8, 0., 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        rec = h.runner.record
        np.testing.assert_allclose(rec['goals_map'], [p[:3] for p in ROUTE], atol=1e-12)
        # 记录里的 odom 点用同一份变换正算回去，必须还原成输入的绝对 map 点
        np.testing.assert_allclose(np.array(rec['goals_odom']) @ h.r.T + h.translation, rec['goals_map'], atol=1e-9)
        for leg in rec['legs']:
            self.assertAlmostEqual(leg['hold_elapsed_s'], 3.0, delta=1e-6)
            self.assertIsNotNone(leg['hold_end_wall'])
        self.assertEqual(rec['end'], 'SUCCEEDED')
        self.assertEqual([leg['end'] for leg in rec['legs']], ['SUCCEEDED'] * len(ROUTE))

    def test_hold_between_points(self):
        h = self.harness(hold=3.0)
        h.place([.8, 0., 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        legs = h.runner.record['legs']
        for i in range(1, len(legs)):
            # 下一点必须在上一段保持结束之后才发
            self.assertGreaterEqual(h.client.goals[i][0], legs[i - 1]['hold_end_wall'] - 1000.0)

    def test_first_point_already_here_is_skipped_but_held(self):
        h = self.harness()
        h.place([.05, -.03, 1.02], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), len(ROUTE) - 1)
        legs = h.runner.record['legs']
        self.assertEqual(len(legs), len(ROUTE))
        self.assertEqual(legs[0]['end'], 'SKIPPED_AT_POINT')
        self.assertIn('已在点上', legs[0]['skip_reason'])
        self.assertIsNone(legs[0]['sent_wall'])
        self.assertAlmostEqual(legs[0]['hold_elapsed_s'], 3.0, delta=1e-6)
        # 首段没发目标，网关还在 READY：不能为此去 reprepare
        self.assertEqual(len(h.prepare_calls), len(ROUTE) - 2)

    def test_near_point_with_wrong_yaw_is_refused(self):
        h = self.harness()
        h.place([.05, -.03, 1.02], yaw_deg=90.)
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.client.goals, [])
        self.assertIn('航向差', h.runner.record['reason'])
    def test_far_first_point_is_refused(self):
        h = self.harness(max_start_offset=2.0)
        h.place([2.5, 0., 1.], yaw_deg=0.)
        # 和边界预检一样，这种"起飞前就该停"的拒绝发生在 freeze() 里，不落 goal_*.json
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.client.goals, [])
        self.assertEqual(h.checked, [])
        self.assertIsNone(h.runner.record)

    def test_custom_points_replace_default(self):
        points = [(0., 0., 1., 0.), (0., 1.5, 1., 90.)]
        h = self.harness(points=points)
        h.place([0., .5, 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        self.assertEqual(len(h.client.goals), 2)
        np.testing.assert_allclose(h.runner.record['goals_map'], [p[:3] for p in points], atol=1e-12)

    def test_check_segments_checks_whole_leg(self):
        h = self.harness(check_segments=True)
        h.place([.8, 0., 1.], yaw_deg=0.)
        self.assertTrue(h.runner.run_lap())
        self.assertTrue(h.checked)
        for path in h.checked:
            self.assertEqual(len(path.poses), 2)

    def test_low_start_prints_height_hint_and_refuses(self):
        h = self.harness()
        h.place([.8, 0., .33], yaw_deg=0.)     # 0.33 m：低于边界 z_min 0.40 + 0.30 m 余量
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.client.goals, [])
        self.assertEqual(h.checked, [])
        # 要明确告诉操作员"先升到 0.70 m 以上"，而不是只留一句 clearance 报错
        self.assertTrue(any('升到 0.70 m 以上' in s for s in h.say_log), h.say_log)

    def test_points_without_yaw_keep_the_frozen_heading(self):
        # test9 的竖直矩形：只给位置，航向继承进入本轮时的机头
        pts = [(2., 0., 1.), (2., -2., 1.), (2., -2., 3.), (2., 0., 3.), (2., 0., 1.)]
        h = self.harness(points=pts, max_start_offset=3.0)
        h.cfg['bounds_max'] = [3., 3., 3.5]        # 与 outdoor_nav_z3.yaml 一致，否则 z=3 会被预检拒绝
        h.place([0., 0., 1.], yaw_deg=30.)
        self.assertTrue(h.runner.run_lap())
        pos = [[g.waypoints[0].pose.position.x, g.waypoints[0].pose.position.y, g.waypoints[0].pose.position.z]
               for _, g in h.client.goals]
        np.testing.assert_allclose(pos,
                                   [[2., 0., 1.], [2., -2., 1.], [2., -2., 3.],
                                    [2., 0., 3.], [2., 0., 1.]], atol=1e-12)
        for _, g in h.client.goals:
            y = goal_yaw_deg(g)
            self.assertAlmostEqual(((y - 30. + 180.) % 360.) - 180., 0., places=6)

    def test_target_footprint_rejection_sends_nothing(self):
        h = self.harness()
        h.place([.8, 0., 1.], yaw_deg=0.)
        h.check_valid = False
        self.assertFalse(h.runner.run_lap())
        self.assertEqual(h.client.goals, [])


class RouteParsingTests(unittest.TestCase):
    def test_parse_valid_point(self):
        self.assertEqual(nav_global.parse_point('2,-2,1,0'), (2., -2., 1., 0.))
        self.assertEqual(nav_global.parse_point(' 0.5, 0.25 , 1.5 , -90 '), (.5, .25, 1.5, -90.))

    def test_parse_three_numbers_means_keep_heading(self):
        self.assertEqual(nav_global.parse_point('2,-2,3'), (2., -2., 3.))
        self.assertEqual(nav_global.parse_point('2,-2,3,90'), (2., -2., 3., 90.))
        for text in ('1,2', '1,2,3,4,5'):
            with self.assertRaises(ValueError):
                nav_global.parse_point(text)

    def test_parse_rejects_bad_input(self):
        for text in ('1,2,3,4,5', 'a,0,1,0', '0,0,nan,0', '0,0,inf,0', '0,0,9,0', '0,0,-2,0'):
            with self.assertRaises(ValueError):
                nav_global.parse_point(text)

    def test_validate_route_legs_and_count(self):
        self.assertEqual(nav_global.validate_route(list(ROUTE)), list(ROUTE))
        with self.assertRaises(ValueError):
            nav_global.validate_route([])
        with self.assertRaises(ValueError):
            nav_global.validate_route([(0., 0., 1., 0.)] * (nav_global.MAX_POINTS + 1))
        with self.assertRaises(ValueError):
            nav_global.validate_route([(0., 0., 1., 0.), (0., .19, 1., 0.)])
        # 同一个点重复出现（原地转向用的情形）会被当成 <0.2 m 段拒掉
        with self.assertRaises(ValueError):
            nav_global.validate_route([(0., 0., 1., 0.), (0., 0., 1., 90.)])


class Test9ConfigTests(unittest.TestCase):
    """test9 的路线最高 3 m：证实必须用 z 上界 3.5 的配置，且它确实放行。"""

    def test_z3_config_is_needed_and_sufficient(self):
        import yaml
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        pts = np.array([[2, 0, 1], [2, -2, 1], [2, -2, 3], [2, 0, 3], [2, 0, 1]], float)

        def allows(name):
            c = yaml.safe_load(open(os.path.join(here, name)))['config']
            lo, hi, m = np.array(c['bounds_min']), np.array(c['bounds_max']), c['goal_boundary_margin']
            return bool(np.all(pts >= lo + m - 1e-9) and np.all(pts <= hi - m + 1e-9))

        self.assertTrue(allows('outdoor_nav_z3.yaml'))
        self.assertFalse(allows('outdoor_nav.yaml'), '默认配置上界 2.20 m，应拒绝 z=3 的点')


if __name__ == '__main__':
    unittest.main()
