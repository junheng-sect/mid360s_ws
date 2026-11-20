#!/usr/bin/env python3
"""test2：冻结一圈基准，前右后左各 1 m；单点成功后保持 5 s。

只通过现有 prepare / execute_route 接口，不切模式、不解锁、不直接发 MAVROS。
实飞每次新的 OFFBOARD 上升沿触发一圈；启动时已经 OFFBOARD 不触发。
--preview 必须对应 real_output=false，按手持/模拟状态等待真实到点判据。
"""
import argparse
import json
import math
import os
import threading
import time

import actionlib
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction, ExecuteRouteGoal
from drone_navigation.core import check_goal_bounds, heading
from nav_forward import State, ACTION_STATE, hint, path_of, rot, say

STOP_REASON = 'task stopped; prepare before next route'
DIRECTIONS = ('前进', '右移', '后退', '左移回起点')
HOLD_SECONDS = 5.0


class Stopped(Exception):
    """本圈不可继续；只有下一次明确触发可以新建一圈。"""


def square_points(position, yaw):
    p = np.asarray(position, dtype=float)
    if p.shape != (3,) or not np.isfinite(p).all() or not math.isfinite(yaw):
        raise Stopped('起点或航向不是有效数值')
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    return np.array([p + forward, p + forward - left, p - left, p.copy()])


class SquareState(State):
    """回调中锁存接管/输入故障，短暂异常恢复也不能让本圈续飞。"""
    def __init__(self):
        self.lock = threading.RLock()
        self.monitor = False
        self.real = False
        self.map_id = ''
        self.issue = ''
        self.entries = 0
        self.last_offboard = False
        super().__init__()

    def stop(self, why):
        with self.lock:
            if self.monitor and not self.issue:
                self.issue = why

    def begin(self, real, map_id):
        with self.lock:
            self.issue = ''
            self.real, self.map_id, self.monitor = real, map_id, True

    def end(self):
        with self.lock:
            self.monitor = False

    def on_fcu(self, m):
        with self.lock:
            off = m.connected and m.armed and m.mode == 'OFFBOARD'
            if off and not self.last_offboard:
                self.entries += 1
            self.last_offboard = off
            super().on_fcu(m)
            if self.real and not off:
                self.stop('已切出 OFFBOARD、上锁或飞控连接断开')

    def on_status(self, m, name):
        with self.lock:
            super().on_status(m, name)
            state = self.st[name]
            if name == 'input_status' and state.get('state') != 'READY':
                self.stop('导航输入失效：' + str(state.get('reason', '')))
            if name == 'reloc' and (state.get('state') != 'LOCALIZED' or state.get('valid') is not True):
                self.stop('重定位失效')
            if name == 'gateway_status':
                if state.get('state') == 'FAULT' and state.get('reason') != STOP_REASON:
                    self.stop('网关故障：' + hint(str(state.get('reason', ''))))
                if self.monitor and state.get('real_output') is not self.real:
                    self.stop('网关输出模式改变')
            if name == 'executor_status' and state.get('state') == 'FAULT':
                self.stop('执行器故障：' + hint(str(state.get('reason', ''))))

    def on_result(self, m):
        with self.lock:
            try:
                result = json.loads(m.data)
                self.result = result if isinstance(result, dict) else {}
            except (ValueError, TypeError):
                self.result = {}
            if self.result.get('confirmed') is not True or (self.monitor and self.result.get('map') != self.map_id):
                self.stop('重定位结果失效或地图改变')

    def on_est(self, m):
        with self.lock:
            super().on_est(m)
            if self.real:
                why = self.ekf_problem()
                if why:
                    self.stop(why)


class SquareRunner:
    test_name = 'test2'
    route_name = '正方形'
    record_prefix = 'square'
    check_segments = False
    side_m = 1.0
    hold_seconds = HOLD_SECONDS
    goals_in_map = False
    repeat_laps = True

    def make_route(self, position, yaw):
        return square_points(position, yaw), np.full(4, yaw), DIRECTIONS

    def make_route_map(self):
        """goals_in_map 子类：直接给 drone_map 绝对航点 (n×3) 与航向 (rad)，不参考当前位置。"""
        raise NotImplementedError('goals_in_map 子类必须实现 make_route_map')

    def skip_leg(self, i, goal, yaw):
        """返回 True 表示这一段不发目标（子类自己记录并保持）。默认不跳过。"""
        return False

    def hold_for(self, i):
        """第 i 段到点后保持多少秒。默认每段都一样，子类可按段类型区分（如原地转向不保持）。"""
        return self.hold_seconds

    def hold_at_point(self, i, leg, seconds):
        self.pause(seconds, stopped=True)

    def __init__(self, args, cfg, state, buf, prepare, check, client):
        self.a, self.cfg, self.s, self.buf = args, cfg, state, buf
        self.prepare, self.check, self.client = prepare, check, client
        self.real = not args.preview
        self.lo = np.asarray(cfg['bounds_min'], float)
        self.hi = np.asarray(cfg['bounds_max'], float)
        self.reference = None
        self.active = False
        self.record = None
        self.record_path = None

    def inputs_problem(self):
        s = self.s
        if s.get('input_status') != 'READY':
            return '导航输入未就绪：' + hint(str(s.get('input_status', 'reason')))
        if s.get('reloc') != 'LOCALIZED' or s.get('reloc', 'valid') is not True:
            return '重定位未就绪'
        if s.result.get('confirmed') is not True or s.result.get('map') != self.cfg['map_id']:
            return '重定位结果与地图不一致'
        p = s.pose()
        if p is None or not np.isfinite(np.r_[p[0], p[1], p[2]]).all():
            return '/drone_nav/odom 断流或数值无效'
        for name, age in (('input_status', .3), ('reloc', 1.5), ('gateway_status', .3)):
            if time.monotonic() - s.st.get(name, {}).get('_at', 0) > age:
                return name + ' 断流'
        if s.get('gateway_status', 'real_output') is not self.real:
            return '网关 real_output 与要求不一致'
        return ''

    def transform(self):
        msg = self.buf.lookup_transform('drone_map', 'drone_odom', rospy.Time(0), rospy.Duration(.05))
        delta = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
        if msg.header.stamp.to_sec() <= 0 or not -.05 <= delta <= .5:
            raise Stopped('drone_map <- drone_odom 变换过期')
        tr = msg.transform
        r = rot(tr.rotation)
        t = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        if not np.isfinite(r).all() or not np.isfinite(t).all():
            raise Stopped('地图变换数值无效')
        return r, t

    def guard(self, stopped=False, preparing=False):
        if rospy.is_shutdown():
            raise Stopped('ROS 已退出')
        why = self.s.issue or self.inputs_problem()
        if why:
            raise Stopped(why)
        if self.real:
            if not self.s.offboard():
                raise Stopped('不在 OFFBOARD 或飞控状态断流')
            why = self.s.ekf_problem()
            if why:
                raise Stopped(why)
        g, reason = self.s.get('gateway_status'), self.s.get('gateway_status', 'reason')
        expected = stopped and g == 'FAULT' and reason == STOP_REASON
        if not (g == 'READY' or expected or (preparing and g == 'PREPARED')):
            raise Stopped('网关 %s：%s' % (g or '无消息', hint(str(reason))))
        if self.s.get('executor_status') == 'FAULT':
            raise Stopped('执行器故障：' + hint(str(self.s.get('executor_status', 'reason'))))
        r, t = self.transform()
        if self.reference is not None:
            r0, t0 = self.reference
            if np.linalg.norm(t - t0) > .01 or np.linalg.norm(r - r0) > .01:
                raise Stopped('地图变换已重置，本圈作废')

    def pause(self, seconds, **guards):
        start = time.monotonic()
        while True:
            self.guard(**guards)
            elapsed = time.monotonic() - start
            if elapsed >= seconds:
                return elapsed
            time.sleep(min(.05, seconds - elapsed))

    def save(self):
        if self.record_path:
            temp = self.record_path + '.tmp'
            with open(temp, 'w') as f:
                json.dump(self.record, f, indent=1, ensure_ascii=False, allow_nan=False)
            os.replace(temp, self.record_path)

    def freeze(self):
        self.guard()
        p, yaw, _ = self.s.pose()
        # 先取地图变换：goals_in_map 的子类要用它把 drone_map 航点反变换到 drone_odom，
        # 而 check_goal_bounds 的边界是 drone_odom 口径，必须用换好的 odom 点检查。
        self.reference = self.transform()
        r, t = self.reference
        direction = r @ np.array([math.cos(yaw), math.sin(yaw), 0.0])
        if np.linalg.norm(direction[:2]) < .9:
            raise Stopped('地图变换不能保持水平航向')
        if self.goals_in_map:
            goals_map, yaw_map, self.names = self.make_route_map()
            goals = (goals_map - t) @ r
            yaws = np.array([heading(r.T, a) for a in yaw_map])
        else:
            goals, yaws, self.names = self.make_route(p, yaw)
            yaw_map = np.array([heading(r, a) for a in yaws])
            goals_map = goals @ r.T + t
        self.yaws = yaws
        try:
            check_goal_bounds(np.vstack([p, goals]), self.cfg,
                              ['起点'] + ['#%d %s' % (i + 1, n) for i, n in enumerate(self.names)])
        except ValueError as error:
            raise Stopped(str(error))
        out = os.path.expanduser(self.a.out or '~/test_data/nav_test/' + time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(out, exist_ok=True)
        self.record_path = os.path.join(out, 'goal_%s_%s_%d.json' % (self.record_prefix, time.strftime('%H%M%S'), time.time_ns()))
        self.record = dict(test=self.test_name, map_id=self.cfg['map_id'], real_output=self.real,
                           trigger='offboard+%gs' % self.a.delay if self.real else 'preview_enter',
                           side_m=self.side_m, hold_s=self.hold_seconds, delay_s=self.a.delay,
                           max_vel=self.cfg['max_vel'], max_acc=self.cfg['max_acc'],
                           start_odom=p.tolist(), start_yaw_deg=math.degrees(yaw),
                           goal_yaw_map_deg=(math.degrees(yaw_map[0]) if self.test_name == 'test2' else np.degrees(yaw_map).tolist()),
                           goal_yaw_odom_deg=np.degrees(yaws).tolist(),
                           goal_boundary_margin=self.cfg.get('goal_boundary_margin', 0.0),
                           bounds_min=self.lo.tolist(), bounds_max=self.hi.tolist(), transform_rotation=r.tolist(),
                           transform_translation=t.tolist(), goals_odom=goals.tolist(), goals_map=goals_map.tolist(),
                           start_wall=time.time(), start_ros=rospy.Time.now().to_sec(), legs=[], end='RUNNING')
        self.save()
        say('本圈起点（drone_odom）%.3f %.3f %.3f，当前航向 %.1f°%s' % (
            *p, math.degrees(yaw), '' if not self.goals_in_map else '；航点用 drone_map 绝对坐标，不随起点平移'))
        for i, (g, gm) in enumerate(zip(goals, goals_map)):
            say('#%d %s：drone_odom (%.3f %.3f %.3f)；drone_map (%.3f %.3f %.3f)' % (i + 1, self.names[i], *g, *gm))
        return goals, goals_map, yaw_map

    def reprepare(self):
        # 成功 action 会发 cancel，让网关进入这个特定 FAULT。必须先看到
        # cancel 已落地，不能用 prepare 抢在 cancel 前清状态再被迟到的 cancel 打断。
        until = time.monotonic() + 2.0
        while True:
            self.guard(stopped=True)
            if (self.s.get('gateway_status') == 'FAULT' and
                    self.s.get('gateway_status', 'reason') == STOP_REASON and
                    self.s.get('executor_status') == 'IDLE'):
                break
            if time.monotonic() >= until:
                raise Stopped('未收到正常的任务结束状态，不清除网关状态')
            time.sleep(.05)
        self.guard(stopped=True)
        response = self.prepare()
        if not response.success:
            raise Stopped('下一段 prepare 被拒：' + hint(response.message))
        # 真实输出在 prepare 后至少 1 s 才 READY；只容忍服务返回前的旧消息。
        until, called = time.monotonic() + 3.0, time.monotonic()
        while True:
            old = self.s.st.get('gateway_status', {}).get('_at', 0) <= called
            self.guard(stopped=old, preparing=True)
            if not old and self.s.get('gateway_status') == 'READY':
                return
            if time.monotonic() >= until:
                raise Stopped('下一段 prepare 后网关未 READY')
            time.sleep(.05)

    def send_leg(self, i, goal, goal_map, yaw_map):
        self.guard()
        # 只检查本段目标，不能把单点检查当作整条轨迹安全证明；轨迹仍由 EGO/执行器检查。
        points = [self.s.pose()[0], goal] if self.check_segments else [goal]
        response = self.check(path_of(points))
        if not response.valid:
            raise Stopped('#%d %s检查拒绝：%s' %
                          (i + 1, '整段路径' if self.check_segments else '目标包络',
                           getattr(response, 'reason', 'unknown/occupied')))
        # This only approves the direct segment at this instant; EGO's actual
        # candidate and its remaining path still need the executor's map lease.
        self.guard()
        ps = PoseStamped()
        ps.header.frame_id = 'drone_map'
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, goal_map)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw_map / 2), math.cos(yaw_map / 2)
        leg = dict(index=i + 1, direction=self.names[i], goal_yaw_map_deg=math.degrees(yaw_map), goal_odom=goal.tolist(), goal_map=goal_map.tolist(),
                   sent_wall=time.time(), sent_ros=rospy.Time.now().to_sec(), end='RUNNING')
        self.record['legs'].append(leg)
        self.save()
        # 设置 active 在 send_goal 之前，Ctrl+C 落在发送中也必须取消。
        self.active = True
        self.client.send_goal(ExecuteRouteGoal(map_id=self.cfg['map_id'], waypoints=[ps]))
        start, next_note, stop_at = time.monotonic(), time.monotonic(), None
        say('#%d/%d %s，目标航向 %.1f°（drone_map）' % (i + 1, len(self.names), self.names[i], math.degrees(yaw_map)))
        while True:
            state = self.client.get_state()
            # 正常 cancel 状态可能比 action result 先到，最多等 0.5 s 接收成功结果。
            self.guard(stopped=True)
            if state == 9:
                break
            if state in (2, 3, 4, 5, 8):
                # Actionlib can receive the terminal status before the result
                # on a different topic. Keep guarding while waiting for it.
                if self.client.get_result() is not None:
                    break
                stop_at = stop_at or time.monotonic()
                if time.monotonic() - stop_at > 1.:
                    raise Stopped('收到任务结束状态，但未收到动作结果')
            elif self.s.get('gateway_status', 'reason') == STOP_REASON:
                stop_at = stop_at or time.monotonic()
                if time.monotonic() - stop_at > .5:
                    raise Stopped('网关已停止，但未收到单点成功结果')
            if time.monotonic() - start > 125:
                raise Stopped('单点任务超时')
            if time.monotonic() >= next_note:
                p = self.s.pose()
                say('#%d｜%s/%s｜离目标 %.3f m｜速度 %.2f m/s' % (
                    i + 1, self.s.get('executor_status'), self.s.get('gateway_status'), np.linalg.norm(goal - p[0]), p[2]))
                next_note = time.monotonic() + 1.0
            time.sleep(.05)
        result = self.client.get_result()
        self.active = False
        p = self.s.pose()
        leg.update(end=ACTION_STATE.get(state, str(state)), reason=result.reason if result else '',
                   elapsed_s=time.monotonic() - start, end_wall=time.time(), end_odom=p[0].tolist(),
                   end_error_m=float(np.linalg.norm(goal - p[0])),
                   end_height_error_m=float(p[0][2] - goal[2]),
                   end_yaw_odom_deg=math.degrees(p[1]), end_ros=rospy.Time.now().to_sec())
        self.save()
        if state != 3 or result is None or result.success is not True:
            raise Stopped('#%d 任务 %s：%s' % (i + 1, leg['end'], hint(leg['reason'])))
        hold = self.hold_for(i)
        say('#%d 稳定到点，保持 %g s' % (i + 1, hold))
        leg.update(hold_start_wall=time.time(), hold_start_ros=rospy.Time.now().to_sec(), hold_elapsed_s=0.0)
        self.save()
        hold_at = time.monotonic()
        try:
            self.hold_at_point(i, leg, hold)
        finally:
            leg['hold_elapsed_s'] = time.monotonic() - hold_at
            leg['hold_end_wall'] = time.time()
            self.save()

    def run_lap(self):
        self.reference, self.record, self.record_path = None, None, None
        self.s.begin(self.real, self.cfg['map_id'])
        try:
            if self.real:
                say('进入 OFFBOARD：保持 %g s 后开始一圈（期间切出即取消）' % self.a.delay)
                self.pause(self.a.delay, preparing=True)
            goals, goals_map, yaw_map = self.freeze()
            sent = 0
            for i in range(len(goals)):
                if self.skip_leg(i, goals[i], self.yaws[i]):
                    continue
                if sent:
                    self.reprepare()
                self.send_leg(i, goals[i], goals_map[i], yaw_map[i])
                sent += 1
            self.record['end'] = 'SUCCEEDED'
            again = '切回定点再切 OFFBOARD 才重跑' if self.repeat_laps else '请飞手接管降落'
            last_leg = self.record['legs'][-1] if self.record['legs'] else {}
            held = last_leg.get('hold_elapsed_s', self.hold_seconds)
            say('%s完成，终点已保持 %.2f s，继续悬停。%s' % (self.route_name, held, again))
            return True
        except BaseException as error:
            cancel_error = ''
            if self.active:
                try:
                    self.client.cancel_goal()
                    until = time.monotonic() + 3.0
                    while self.client.get_state() not in (2, 3, 4, 5, 8, 9) and time.monotonic() < until:
                        time.sleep(.05)
                except Exception as failure:
                    cancel_error = '；取消请求异常：' + str(failure)
                self.active = False
            why = ('Ctrl+C，停止本圈' if isinstance(error, KeyboardInterrupt) else str(error)) + cancel_error
            if self.record:
                self.record.update(end='INTERRUPTED' if isinstance(error, KeyboardInterrupt) else 'ABORTED', reason=why)
                if self.record['legs'] and self.record['legs'][-1]['end'] == 'RUNNING':
                    self.record['legs'][-1].update(end=self.record['end'], reason=why, end_wall=time.time())
            again = '切回定点再切 OFFBOARD 才重跑' if self.repeat_laps else '请飞手接管降落'
            say('本圈终止：%s。不会续飞剩余边；%s' % (why, again))
            if not isinstance(error, Stopped):
                raise
            return False
        finally:
            self.s.end()
            if self.record:
                self.record['end_wall'] = time.time()
                self.record['end_ros'] = rospy.Time.now().to_sec()
                self.save()
                say('本圈记录：' + self.record_path)

    def automatic(self):
        # 先取当前序号，启动时已经 OFFBOARD 的那一沿永远不触发。
        while self.s.fcu is None and not rospy.is_shutdown():
            time.sleep(.1)
        seen = self.s.entries
        if self.s.offboard():
            say('启动时已经 OFFBOARD：不触发，先切回定点再切回来')
        last_try, last_note = -math.inf, None
        while not rospy.is_shutdown():
            if self.s.entries > seen:
                seen = self.s.entries
                if self.s.offboard():
                    self.run_lap()
                    if not self.repeat_laps:
                        return
                    # 飞行/取消期间发生的边沿属于本圈，不能在返回这里后意外触发新圈。
                    seen = self.s.entries
                    continue
            if self.s.offboard():
                note = '等待切回定点再切 OFFBOARD；不会自动再次发目标'
            else:
                # 待命阶段允许显式准备；本圈内部仅 reprepare 能清正常结束状态。
                if time.monotonic() - last_try >= 1.0:
                    last_try = time.monotonic()
                    try:
                        response = self.prepare()
                        why = '' if response.success else hint(response.message)
                    except Exception as error:
                        why = str(error)
                    note = '待命：网关 %s %s｜%s' % (self.s.get('gateway_status') or '无消息',
                                                         hint(str(self.s.get('gateway_status', 'reason'))),
                                                         why or self.s.ekf_problem() or '起飞悬停后切 OFFBOARD')
                else:
                    note = last_note
            if note and note != last_note:
                say(note)
            last_note = note
            time.sleep(.05)

    def preview(self):
        if self.s.get('gateway_status', 'real_output') is not False:
            raise Stopped('预览要求网关 real_output=false，不允许真实输出')
        response = self.prepare()
        if not response.success:
            raise Stopped('预览 prepare 被拒：' + hint(response.message))
        until = time.monotonic() + 5.0
        while self.s.get('gateway_status') != 'READY':
            if rospy.is_shutdown() or time.monotonic() >= until:
                raise Stopped('预览 prepare 后网关未 READY')
            time.sleep(.05)
        return self.run_lap()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--delay', type=float, default=3.0, help='OFFBOARD 后出发延时，默认 3 s')
    ap.add_argument('--preview', action='store_true', help='只预览一圈，必须 real_output=false')
    ap.add_argument('--out', default='', help='目标记录目录，由 nav_test2.sh 设置')
    args = ap.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.delay) or not 0 < args.delay <= 30:
        ap.error('--delay 要在 0~30 s 之间（不含 0）')
    rospy.init_node('nav_square', anonymous=True, disable_signals=True)
    while not rospy.has_param('/drone_nav/config') and not rospy.is_shutdown():
        say('等 navigation.launch 加载 /drone_nav/config ...')
        time.sleep(1)
    cfg, state = rospy.get_param('/drone_nav/config'), SquareState()
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)  # 保留 listener，整个运行期间收 TF
    rospy.wait_for_service('/drone_nav/prepare')
    client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
    client.wait_for_server()
    runner = SquareRunner(args, cfg, state, buf, rospy.ServiceProxy('/drone_nav/prepare', Trigger),
                          rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory), client)
    while not rospy.is_shutdown():
        why = runner.inputs_problem()
        if not why:
            break
        # 输出模式不符不能等待它自行改变，更不能发送目标。
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
        say('自动发目标已停；定位链路由外层脚本保留，请人工切回定点')
        raise SystemExit(130)
    except Stopped as error:
        say(str(error))
        raise SystemExit(1)
