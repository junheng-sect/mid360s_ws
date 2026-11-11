#!/usr/bin/env python3
"""test7：按 drone_map 绝对（全局）坐标飞航点，每点稳定到达后保持 N 秒（默认 3 s）。

与 test2/4/5/indoor 的区别：航点不再是"相对当前位姿的偏移"，而是直接给出的 drone_map 绝对坐标。
任务层 /drone_nav/execute_route 本来就只收 drone_map 帧（mission.py：frame_id!='drone_map' 直接拒），
所以这里把绝对点原样交给它，不再做"当前位置 + 偏移 → map"的换算；只为了边界检查和记录，把点反算一次
drone_odom。

航点帧 drone_map（室外 FLU）：
  x = 建图时机头方向，y = 左，z = 上。
  原点 = 建图/重定位启动时机体（雷达 IMU）所在位置，也就是**飞机停在地面时的机体高度**，
  不是大地/地面海拔。z=1.0 表示机体离地约 1 m（腿和桨保在机体下方约 0.15 m）。
  car_bridge 的 map 帧把地面定在 z=0，比这个帧低 0.151 m（map_frame_outdoor.yaml，未实测）。

默认航点（drone_map，单位 m，第四个数是偏航角 °）：
  (0,0,1,0°) (2,0,1,0°) (2,-2,1,0°) (0,-2,1,0°) (0,0,1,0°)

流程（一般由 scripts/nav_test7.sh 调用，重定位和导航已经起好）：
  1) 人工定点起飞到约 1 m 悬停，切 OFFBOARD；延时结束后打印当前位置与全部航点（odom/map 两套）
  2) 每个点单独发一次 action；到点判据由任务层给：位置 ≤0.15 m、速度 ≤0.1 m/s、航向 ≤5° 连续 1 s
  3) 到点后保持 hold 秒，再 prepare 下一段
  4) 首点如果已经在脚下（<0.2 m 且航向差 ≤5°）不发目标、只保持：任务层拒绝离当前位置 <0.2 m 的新目标
     （原地转向只在"重复同一终点"时放行），而人工起飞点通常就在首点正下方
  5) 任何接管、上锁、输入/重定位失效、执行器或网关故障、地图变换重置、Ctrl+C 都终止本轮，不续飞

用法：
  nav_global.py [--point x,y,z,yaw_deg ...] [--hold 3] [--delay 3]
                [--max-start-offset 2] [--check-segments] [--preview] [--out DIR]
"""
import argparse
import math
import sys
import time

import actionlib
import numpy as np
import rospy
import tf2_ros
from std_srvs.srv import Trigger
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction

from nav_forward import say  # noqa: E402  同目录
from nav_square import SquareRunner, SquareState, Stopped  # noqa: E402
from drone_navigation.core import heading  # noqa: E402  纯 numpy，与网关同一套航向换算

# drone_map 绝对航点：x, y, z (m), yaw (deg)
DEFAULT_ROUTE = ((0., 0., 1., 0.), (2., 0., 1., 0.), (2., -2., 1., 0.),
                 (0., -2., 1., 0.), (0., 0., 1., 0.))
DEFAULT_HOLD = 3.0
MIN_LEG_M = 0.2          # 任务层拒绝 <0.2 m 的新目标
MAX_POINTS = 50
SKIP_NEAR_M = 0.20       # 已经在这个点上：不发目标，只保持
SKIP_YAW_DEG = 5.0
Z_RANGE = (-1.0, 5.0)


def parse_point(text):
    """'x,y,z[,yaw_deg]' -> (x, y, z[, yaw_deg])；省略航向时该点保持当前机头不动。"""
    fields = text.split(',')
    if len(fields) not in (3, 4):
        raise ValueError('--point 要 3 或 4 个数 "x,y,z" 或 "x,y,z,yaw_deg"，收到 %r' % text)
    try:
        values = tuple(float(x) for x in fields)
    except ValueError:
        raise ValueError('--point 里有非数字：%r' % text)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('--point 里有非有限值：%r' % text)
    if not Z_RANGE[0] <= values[2] <= Z_RANGE[1]:
        raise ValueError('--point 的 z=%.3f 超出 %g~%g m（drone_map 口径）' % (values[2], *Z_RANGE))
    return values


def validate_route(points):
    """点数与相邻间距；边界留给 check_goal_bounds（它要换算后的 drone_odom 点）。"""
    if not 1 <= len(points) <= MAX_POINTS:
        raise ValueError('航点数量要在 1~%d 之间，现在 %d 个' % (MAX_POINTS, len(points)))
    for i in range(1, len(points)):
        gap = math.dist(tuple(points[i][:3]), tuple(points[i - 1][:3]))
        if gap < MIN_LEG_M:
            raise ValueError('第 %d 段只有 %.3f m（< %.2f m），任务层会拒绝：%r → %r'
                             % (i, gap, MIN_LEG_M, points[i - 1], points[i]))
    return points


def route_names(points):
    out = []
    for i, p in enumerate(points):
        if len(p) > 3:
            out.append('#%d (%.2f, %.2f, %.2f) yaw %.0f°' % (i + 1, p[0], p[1], p[2], p[3]))
        else:
            out.append('#%d (%.2f, %.2f, %.2f) 航向保持' % (i + 1, p[0], p[1], p[2]))
    return tuple(out)


class GlobalRunner(SquareRunner):
    test_name = 'test7'
    route_name = '全局航点（drone_map 绝对坐标）'
    record_prefix = 'test7'
    goals_in_map = True

    def __init__(self, args, cfg, state, buf, prepare, check, client):
        super().__init__(args, cfg, state, buf, prepare, check, client)
        self.points = validate_route(args.points)
        self.hold_seconds = float(args.hold)
        self.check_segments = bool(args.check_segments)
        if getattr(args, 'record_prefix', ''):
            self.record_prefix = str(args.record_prefix)

    def make_route_map(self):
        """drone_map 绝对航点 (n×3)、航向 (rad)、名字；顺带核对首点离当前位置多远。"""
        pose = self.s.pose()
        if pose is None:
            raise Stopped('/drone_nav/odom 断流：无法核对当前位置')
        r, t = self.reference
        here_map = r @ pose[0] + t
        first = np.array(self.points[0][:3], float)
        gap = float(np.linalg.norm(first - here_map))
        say('当前位置 drone_map (%.3f %.3f %.3f)、drone_odom (%.3f %.3f %.3f)；首点 #1 离这里 %.3f m'
            % (*here_map, *pose[0], gap))
        say('航点按 drone_map 绝对坐标发（z 以飞机着地时的机体高度为 0，不是地面海拔）')
        # 边界（含 goal_boundary_margin）是发目标前预检的口径：当前位置越界时先把原因和做法说清楚，
        # 免得只看到一句 "z_min clearance ... < required ..."（11-11 第一次实飞就是这样被拒的）。
        margin = float(self.cfg.get('goal_boundary_margin', 0.0))
        lo, hi = self.lo + margin, self.hi - margin
        outside = [(axis, float(pose[0][k])) for k, axis in enumerate('xyz')
                   if not lo[k] <= pose[0][k] <= hi[k]]
        if outside:
            low = any(axis == 'z' and value < lo[2] for axis, value in outside)
            say('注意：当前位置 %s 不在允许范围（x/y %.2f~%.2f m、z %.2f~%.2f m ＝ 边界加 %.2f m 余量），'
                '本轮会在预检处被拒绝；%s'
                % (', '.join('%s=%.3f' % item for item in outside), lo[0], hi[0], lo[2], hi[2], margin,
                   '先把飞机升到 %.2f m 以上再切 OFFBOARD' % lo[2] if low
                   else '先把飞机挪回范围内再切 OFFBOARD'))
        if gap > self.a.max_start_offset:
            raise Stopped('首点 #1 离当前位置 %.2f m，超过 --max-start-offset %.2f m：'
                          '先核对地图、重定位原点和航点，确认不是拿错地图再飞'
                          % (gap, self.a.max_start_offset))
        # 返回值约定：航向按 drone_map 弧度给（freeze 再换算成 odom）。
        # 没写航向的点用"当前机头在 drone_map 下的航向"，即保持不动、不主动转向。
        here_yaw_map = heading(r, pose[1])
        yaws = [here_yaw_map if len(p) < 4 else math.radians(p[3]) for p in self.points]
        return (np.array([p[:3] for p in self.points], float),
                np.array(yaws),
                route_names(self.points))

    def skip_leg(self, i, goal, yaw):
        """首点（或后续点）已经在脚下就不发目标，只保持 hold 秒。"""
        pose = self.s.pose()
        if pose is None:
            return False
        gap = float(np.linalg.norm(pose[0] - goal))
        dyaw = abs(math.atan2(math.sin(pose[1] - yaw), math.cos(pose[1] - yaw)))
        if gap >= SKIP_NEAR_M:
            return False
        if dyaw > math.radians(SKIP_YAW_DEG):
            raise Stopped('第 %d 点离当前位置只有 %.3f m，但航向差 %.1f°：任务层只放行"重复同一终点"的原地转向，'
                          '先把飞机挪开 ≥%.2f m，或者改掉这个点'
                          % (i + 1, gap, math.degrees(dyaw), MIN_LEG_M))
        leg = dict(index=i + 1, direction=self.names[i],
                   goal_yaw_map_deg=self.record['goal_yaw_map_deg'][i],
                   goal_odom=self.record['goals_odom'][i], goal_map=self.record['goals_map'][i],
                   sent_wall=None, sent_ros=None, end='SKIPPED_AT_POINT',
                   skip_reason='已在点上：离 %.3f m、航向差 %.1f°，不发目标' % (gap, math.degrees(dyaw)),
                   hold_start_wall=time.time(), hold_start_ros=rospy.Time.now().to_sec(), hold_elapsed_s=0.0)
        self.record['legs'].append(leg)
        self.save()
        say('#%d/%d 已经在点上（离 %.3f m、航向差 %.1f°）：不发目标，保持 %g s'
            % (i + 1, len(self.points), gap, math.degrees(dyaw), self.hold_seconds))
        hold_at = time.monotonic()
        try:
            self.pause(self.hold_seconds, stopped=True)
        finally:
            leg['hold_elapsed_s'] = time.monotonic() - hold_at
            leg['hold_end_wall'] = time.time()
            self.save()
        return True


def build_parser(description=None, hold_default=DEFAULT_HOLD, owner='nav_test7.sh'):
    ap = argparse.ArgumentParser(description=description or __doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--point', action='append', default=[], metavar='x,y,z,yaw_deg',
                    help='drone_map 绝对航点（可重复）；给了就替换默认 5 点，按给出顺序飞')
    ap.add_argument('--hold', type=float, default=hold_default,
                    help='每个目标到达后保持多少秒，默认 %g' % hold_default)
    ap.add_argument('--delay', type=float, default=3.0, help='切 OFFBOARD 后多久开始发第一个目标，默认 3 s')
    ap.add_argument('--max-start-offset', type=float, default=2.0, metavar='M',
                    help='当前位置离首点超过这么多米就拒绝开跑（防拿错地图/原点），默认 2')
    ap.add_argument('--check-segments', action='store_true',
                    help='每段出发前检查当前位置到目标的整段直线（默认只查目标足迹）')
    ap.add_argument('--preview', action='store_true', help='只预览一轮，必须 real_output=false')
    ap.add_argument('--out', default='', help='记录目录，由 %s 设置' % owner)
    ap.add_argument('--record-prefix', default='', metavar='NAME',
                    help='目标记录文件名前缀（默认用 runner 自己的，如 test7；多个测试共用一个 runner 时用它区分）')
    return ap


def parse_args(ap):
    """解析并校验航点；返回命名空间，args.points 已是 4 元组列表。"""
    args = ap.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.delay) or not 0 < args.delay <= 30:
        ap.error('--delay 要在 0~30 s 之间（不含 0）')
    if not math.isfinite(args.hold) or not 0 <= args.hold <= 60:
        ap.error('--hold 要在 0~60 s 之间')
    if not math.isfinite(args.max_start_offset) or args.max_start_offset <= 0:
        ap.error('--max-start-offset 要是正数')
    try:
        args.points = ([parse_point(t) for t in args.point] if args.point
                       else [tuple(p) for p in DEFAULT_ROUTE])
        validate_route(args.points)
    except ValueError as error:
        ap.error(str(error))
    return args


def serve(node, args, runner_cls, state_cls=SquareState):
    """起节点、等输入就绪；--preview 跑一轮预览，否则进入"等 OFFBOARD 上升沿"的自动循环。"""
    rospy.init_node(node, anonymous=True, disable_signals=True)
    while not rospy.has_param('/drone_nav/config') and not rospy.is_shutdown():
        say('等 navigation.launch 加载 /drone_nav/config ...')
        time.sleep(1)
    cfg, state = rospy.get_param('/drone_nav/config'), state_cls()
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)  # noqa: F841  整个运行期间收 TF
    rospy.wait_for_service('/drone_nav/prepare')
    client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
    client.wait_for_server()
    runner = runner_cls(args, cfg, state, buf, rospy.ServiceProxy('/drone_nav/prepare', Trigger),
                        rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory), client)
    while not rospy.is_shutdown():
        why = runner.inputs_problem()
        if not why:
            break
        # 输出模式不符不能等它自行改变，更不能发送目标。
        if state.get('gateway_status', 'real_output') in (True, False) and \
                state.get('gateway_status', 'real_output') is not runner.real:
            raise Stopped(why)
        say('等输入和重定位：' + why)
        time.sleep(1)
    if args.preview:
        return 0 if runner.preview() else 1
    runner.automatic()
    return 0


def main():
    return serve('nav_global', parse_args(build_parser()), GlobalRunner)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        say('自动发目标已停；定位链路保留，请人工切回定点')
        raise SystemExit(130)
    except Stopped as error:
        say(str(error))
        raise SystemExit(1)
