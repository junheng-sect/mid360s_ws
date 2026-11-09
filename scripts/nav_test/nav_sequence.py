#!/usr/bin/env python3
"""test4/test5/indoor1/indoor2：通过现有导航 action 执行，不直接发布 MAVROS 设定值。"""
import argparse
import math
import time

import actionlib
import numpy as np
import rospy
import tf2_ros
from std_srvs.srv import Trigger
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction
from nav_forward import say
from nav_square import SquareRunner, SquareState, Stopped


def sequence_points(test, position, yaw):
    p = np.asarray(position, dtype=float)
    if p.shape != (3,) or not np.isfinite(p).all() or not math.isfinite(yaw):
        raise Stopped('起点或航向不是有效数值')
    f = np.array([math.cos(yaw), math.sin(yaw), 0.])
    if test == 'test_indoor1':
        return np.array([p + f]), np.array([yaw]), ('前进 1 m',)
    if test == 'test_indoor2':
        right = np.array([math.sin(yaw), -math.cos(yaw), 0.])
        return (np.array([p+.5*f, p+.5*(f+right), p+.5*right, p.copy()]),
                np.full(4, yaw), ('前进 0.5 m', '右移 0.5 m', '后退 0.5 m', '左移 0.5 m 回起点'))
    if test == 'test4':
        return (np.array([p + 2*f, p, p + 4*f, p]), np.full(4, yaw),
                ('前进 2 m', '返回起点（2 m）', '前进 4 m', '返回起点（4 m）'))
    if test != 'test5':
        raise Stopped('未知测试 ' + str(test))
    up = np.array([0., 0., .5])
    goals = np.array([p+up, p, p+f+up, p, p, p, p, p])
    yaws = np.r_[np.full(4, yaw), yaw + np.arange(1, 5)*math.pi/2]
    return goals, yaws, ('升高 0.5 m', '降回起始高度', '前进 1 m 同时升高 0.5 m',
                         '斜着返回起点', '原地左转 90°', '原地左转 180°',
                         '原地左转 270°', '原地左转 360°')


class SequenceRunner(SquareRunner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.test_name = self.a.test
        self.route_name = {'test4': '长距离往返', 'test5': '高度、斜飞和原地航向',
                           'test_indoor1': '室内前进 1 m', 'test_indoor2': '室内 0.5 m 正方形'}[self.a.test]
        self.record_prefix = self.a.test
        self.check_segments = self.a.test in ('test5', 'test_indoor1', 'test_indoor2')
        self.side_m = .5 if self.a.test == 'test_indoor2' else None
        if self.a.test == 'test_indoor2':
            self.hold_seconds = 2.0

    def make_route(self, position, yaw):
        return sequence_points(self.test_name, position, yaw)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--test', choices=('test4', 'test5', 'test_indoor1', 'test_indoor2'), required=True)
    ap.add_argument('--delay', type=float, default=3.)
    ap.add_argument('--preview', action='store_true')
    ap.add_argument('--out', default='')
    args = ap.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.delay) or not 0 < args.delay <= 30:
        ap.error('--delay 要在 0~30 s 之间（不含 0）')
    rospy.init_node('nav_' + args.test, anonymous=True, disable_signals=True)
    while not rospy.has_param('/drone_nav/config') and not rospy.is_shutdown():
        time.sleep(.2)
    state, buf = SquareState(), tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    rospy.wait_for_service('/drone_nav/prepare')
    client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
    client.wait_for_server()
    runner = SequenceRunner(args, rospy.get_param('/drone_nav/config'), state, buf,
                            rospy.ServiceProxy('/drone_nav/prepare', Trigger),
                            rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory), client)
    while not rospy.is_shutdown():
        why = runner.inputs_problem()
        if not why:
            break
        if state.get('gateway_status', 'real_output') in (True, False) and state.get('gateway_status', 'real_output') is not runner.real:
            raise Stopped(why)
        say('等输入和重定位：' + why)
        time.sleep(1)
    if args.preview:
        return 0 if runner.preview() else 1
    runner.automatic()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        say('自动发目标已停；定位链路保留，请人工切回定点')
        raise SystemExit(130)
    except Stopped as error:
        say(str(error))
        raise SystemExit(1)
