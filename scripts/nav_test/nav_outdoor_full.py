#!/usr/bin/env python3
"""test10: one manually armed outdoor flight, automatic takeoff/route/slow OFFBOARD landing.

No arming publisher/service, no automatic retry of a flight. --dry-run uses no ROS.
The six map goals intentionally turn only after translation (existing executor).
"""
import argparse
import json
import math
import os
import sys
import time
import threading

ROUTE = ((2., 0., 1., -90.), (2., -2., 1., -90.),
         (2., -2., 2., 0.), (0., -2., 2., 0.),
         (0., -2., 1., 0.), (0., 0., 1., 0.))
HOLDS = (0., 0., 1., 0., 1., 2.)
NAMES = ('前进后右转90°', '朝前沿-y前进', '上升后左转90°',
         '保持机头后退', '原地下降', '向左侧飞返回')
TAKEOFF_TIMEOUT = 20.
LEG_TIMEOUT = 20.
LAND_TIMEOUT = 40.  # ~1.1 m at 0.1 m/s, plus the prepare/READY window and COM_DISARM_LAND
STABLE_SECONDS = 2.
SERVICE_TIMEOUT = 2.
EXIT_REASON = 'OFFBOARD exited; explicit prepare/start required'


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--delay', type=float, default=3.)
    p.add_argument('--preview', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--nav-config', default=os.path.join(os.path.dirname(__file__), 'outdoor_nav_yaw45.yaml'),
                   help='--dry-run reads this file; real run uses /drone_nav/config')
    p.add_argument('--out', default='')
    return p


def arguments(argv=None):
    p = parser()
    a = p.parse_args(argv)
    if not math.isfinite(a.delay) or not 1. <= a.delay <= 4.:
        p.error('--delay 要在 1~4 s 之间')
    return a


def profile(c):
    # Reject overrides that would silently change the approved flight envelope.
    expected = dict(takeoff_height=1., yaw_rate_deg=45., max_vel=1., max_acc=1.,
                    max_vel_z=.3, max_acc_z=.3)
    for k, v in expected.items():
        if not math.isfinite(float(c.get(k, float('nan')))) or abs(float(c[k]) - v) > 1e-9:
            raise ValueError('test10 要求 %s=%g，收到 %r' % (k, v, c.get(k)))
    if not c.get('map_id'):
        raise ValueError('map_id 为空')


def dry_run(a):
    import yaml
    with open(a.nav_config) as f:
        c = yaml.safe_load(f)['config']
    profile(c)
    print('test10 DRY RUN：不初始化 ROS、不启动链路、不写运行记录')
    print('配置：%s；地图：%s' % (a.nav_config, c['map_id']))
    print('人工解锁 → 等 %g s → OFFBOARD 自动起飞1 m → 实测稳定2 s' % a.delay)
    for i, (p, hold) in enumerate(zip(ROUTE, HOLDS), 1):
        print('#%d %s (%g,%g,%g,%g°)，额外保持%g s' % (i, NAMES[i-1], *p, hold))
    print('返回稳定 → prepare 后 /drone_nav/land 慢速下降（0.1 m/s）→ 新鲜着地/上锁确认；只飞一圈，不自动解锁')
    print('水平8 m，范围2×2 m，高度1–2 m；yaw45°/s，上升0.3 m/s、0.3 m/s²，降落0.1 m/s')
    print('预计解锁至上锁70–90 s（非保证）；单起飞/航段20 s，慢速降落最多%g s' % LAND_TIMEOUT)
    if a.preview:
        print('PREVIEW：仅空中航线，不验证起飞/降落，不调用切模式服务')


# Allow a dry run or help without importing ROS, NumPy or creating subscribers.
if __name__ == '__main__' and ('--dry-run' in sys.argv or '-h' in sys.argv or '--help' in sys.argv):
    early = arguments()
    if early.dry_run:
        dry_run(early)
    raise SystemExit(0)

import numpy as np
import rospy
import actionlib
import tf2_ros
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import SetMode
from std_srvs.srv import Trigger
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction
from drone_navigation.core import heading, wrap, check_goal_bounds
from nav_forward import State, path_of, say
from nav_square import SquareState, SquareRunner, Stopped, STOP_REASON
from nav_global import GlobalRunner
from nav_land import Fc, precheck


class FullState(SquareState):
    """Latch transient faults; allow only the explicitly requested mode transitions."""
    def __init__(self):
        self.phase = 'STARTUP'
        self.fc = None
        self.initial_armed = None
        self.arm_at = None
        self.land_requested = False
        self.land_confirmed_at = None
        self.disarmed_at = None
        self.offboard_entered_at = None
        super().__init__()

    def on_fcu(self, m):
        with self.lock:
            old = self.fcu
            if self.initial_armed is None:
                self.initial_armed = bool(m.armed)
            State.on_fcu(self, m)
            if not self.monitor or not self.real:
                return
            if not m.connected:
                self.stop('飞控连接断开')
            landing = self.phase in ('LAND_REQUEST', 'LANDING')
            if landing:
                # The slow descent stays in OFFBOARD; only the pilot leaving it while still armed is a takeover.
                if m.armed and m.mode != 'OFFBOARD':
                    self.stop('降落期间飞手接管/模式改变：' + m.mode)
                if not m.armed:
                    self.disarmed_at = self.disarmed_at or time.monotonic()
                elif self.disarmed_at is not None:
                    self.stop('降落后重新解锁，本轮不重跑')
                return
            allowed = ('POSCTL', 'OFFBOARD') if self.phase == 'OFFBOARD_ENTRY' else (
                ('OFFBOARD',) if self.phase in ('TAKEOFF', 'ROUTE') else ('POSCTL',))
            if m.mode not in allowed:
                self.stop('飞手接管/模式改变：' + m.mode)
            if self.phase == 'OFFBOARD_ENTRY':
                if m.mode == 'OFFBOARD':
                    self.offboard_entered_at = self.offboard_entered_at or time.monotonic()
                elif self.offboard_entered_at is not None:
                    self.stop('切入 OFFBOARD 后又切出，本轮终止')
            if self.phase == 'PREFLIGHT' and m.armed:
                self.stop('预检未结束就解锁，不触发起飞')
            if self.phase == 'WAIT_ARM' and old is not None and not old.armed and m.armed:
                self.arm_at = time.monotonic()
            if self.phase in ('COUNTDOWN', 'OFFBOARD_ENTRY', 'TAKEOFF', 'ROUTE') and not m.armed:
                self.stop('飞控上锁，本轮终止')
            if self.phase == 'WAIT_ARM' and self.arm_at is not None and not m.armed:
                self.stop('解锁后又上锁，本轮终止')

    def ekf_problem(self):
        why = State.ekf_problem(self)
        ground = self.fc is not None and self.fc.on_ground()
        ground_phase = self.phase in ('STARTUP', 'PREFLIGHT', 'WAIT_ARM', 'COUNTDOWN',
                                      'OFFBOARD_ENTRY', 'TAKEOFF', 'LAND_REQUEST', 'LANDING')
        const_only = why == '飞控 EKF 处于 const_pos（飞机静止在地面时 PX4 就报这个，起飞后消失）'
        if ground and ground_phase and const_only:
            return ''
        # PX4 raises const_pos a moment before its landed state flips at touchdown: within 5 cm of the
        # recorded ground reference during the descent this is the ground, not an estimate fault.
        h = self.fc.agl() if self.fc is not None else None
        if const_only and self.phase in ('LAND_REQUEST', 'LANDING') and h is not None and h <= .05:
            return ''
        return why

    def on_est(self, m):
        with self.lock:
            State.on_est(self, m)
            if self.monitor and self.real:
                why = self.ekf_problem()
                if why:
                    self.stop(why)

    def on_status(self, m, name):
        with self.lock:
            if name != 'gateway_status':
                SquareState.on_status(self, m, name)
                return
            State.on_status(self, m, name)
            d = self.st[name]
            # The gateway owns the slow descent, so its state replaces the old AUTO.LAND confirmation.
            land = d.get('land')
            if self.land_requested and self.phase in ('LAND_REQUEST', 'LANDING'):
                if land in ('DESCENDING', 'TOUCHED'):
                    self.land_confirmed_at = self.land_confirmed_at or time.monotonic()
                elif land == 'STOPPED' and self.land_confirmed_at is not None and not (
                        self.fc is not None and self.fc.on_ground() and
                        self.fcu is not None and not self.fcu.armed):
                    self.stop('降落中止：网关停止下降')
            if d.get('state') == 'FAULT':
                reason = d.get('reason', '')
                normal = reason == STOP_REASON
                normal_exit = (reason == EXIT_REASON and self.land_requested and
                               self.phase in ('LAND_REQUEST', 'LANDING'))
                resting = (reason == 'PX4 local estimate unavailable' and self.fc is not None and
                           self.fc.on_ground() and not self.ekf_problem() and
                           self.phase in ('PREFLIGHT', 'WAIT_ARM', 'LANDING'))
                if not (normal or normal_exit or resting):
                    self.stop('网关故障：' + reason)
            if self.monitor and d.get('real_output') is not self.real:
                self.stop('网关输出模式改变')


class FullRunner(GlobalRunner):
    test_name = 'test10'
    route_name = '完整室外综合测试'
    record_prefix = 'test10'

    def __init__(self, args, cfg, state, buf, prepare, check, client, fc,
                 prepare_takeoff=None, takeoff=None, land=None, set_mode=None):
        args.points, args.hold, args.check_segments, args.max_start_offset = list(ROUTE), 0., False, 3.
        super().__init__(args, cfg, state, buf, prepare, check, client)
        self.fc = fc
        self.s.fc = fc
        self.prepare_takeoff, self.takeoff, self.set_mode = prepare_takeoff, takeoff, set_mode
        self.used = False
        self.flight = None
        self.flight_path = None
        self.ground_z = None
        self.deadline = None
        self.current = None
        self.holding = False
        self.stage_deadline = None
        self.prepare = lambda: self.service_call('prepare', prepare, stopped=True, preparing=True)
        self.check = lambda path: self.service_call('check_trajectory', check, path)
        self.prepare_takeoff = lambda: self.service_call('prepare_takeoff', prepare_takeoff)
        self.takeoff = lambda: self.service_call('takeoff', takeoff)
        self.land_start = lambda: self.service_call('land', land)
        self.set_mode = lambda **kw: self.service_call('set_mode', set_mode, **kw)

    def service_call(self, name, service, *args, stopped=False, preparing=False, **kwargs):
        """Bound a stalled RPC without retrying an ambiguously delivered request.

        ROS cannot revoke an already delivered RPC. On timeout/guard failure the
        daemon may finish that request, but it cannot issue ANY follow-up command.
        """
        self.guard(stopped=stopped, preparing=preparing)
        done, abandoned, result = threading.Event(), threading.Event(), {}
        start = time.monotonic()
        until = min(start + SERVICE_TIMEOUT, self.stage_deadline or math.inf,
                    self.deadline or math.inf)
        event = dict(service=name, request=kwargs, wall=time.time(), phase=self.s.phase)
        if self.flight is not None:
            self.flight.setdefault('services', []).append(event)
            self.save()

        def deliver():
            try:
                if not abandoned.is_set():
                    result['reply'] = service(*args, **kwargs)
            except Exception as e:
                result['error'] = e
            finally:
                done.set()

        threading.Thread(target=deliver, daemon=True).start()
        try:
            while not done.wait(.01):
                self.guard(stopped=stopped, preparing=preparing)
                if time.monotonic() >= until:
                    raise Stopped('%s 服务调用超时，本轮终止，不重发请求' % name)
                time.sleep(.02)
            self.guard(stopped=stopped, preparing=preparing)
            if time.monotonic() >= until:
                raise Stopped('%s 服务调用超时，本轮终止，不重发请求' % name)
            if 'error' in result:
                raise Stopped('%s 服务调用失败：%s' % (name, result['error']))
            reply = result['reply']
            event['result'] = {k: getattr(reply, k) for k in ('success', 'message', 'mode_sent', 'valid', 'reason')
                               if hasattr(reply, k)}
            return reply
        except BaseException as e:
            abandoned.set()
            event['error'] = str(e)
            raise
        finally:
            event.update(end_wall=time.time(), elapsed_s=time.monotonic()-start)
            self.save()

    def snapshot(self):
        p, fp, f = self.s.pose(), self.fc.pos(), self.s.fcu
        valid_pose = p is not None and np.isfinite(np.r_[p[0], p[1], p[2]]).all()
        return dict(mode=f.mode if f else None, armed=bool(f.armed) if f else None,
                    landed=self.fc.landed if self.fc.ext_fresh() else None,
                    fc_position=fp.tolist() if fp is not None and np.isfinite(fp).all() else None,
                    odom_position=p[0].tolist() if valid_pose else None,
                    speed=p[2] if valid_pose else None,
                    gateway=self.s.get('gateway_status'), takeoff=self.s.get('gateway_status', 'takeoff'))

    def save(self):
        super().save()
        if self.flight_path:
            temp = self.flight_path + '.tmp'
            with open(temp, 'w') as f:
                json.dump(self.flight, f, indent=1, ensure_ascii=False, allow_nan=False)
            os.replace(temp, self.flight_path)

    def phase(self, name):
        with self.s.lock:
            self.s.phase = name
        if self.flight is not None:
            self.flight['phases'].append(dict(phase=name, wall=time.time(), ros=rospy.Time.now().to_sec(),
                                              state=self.snapshot()))
            self.save()
        say('test10 阶段：' + name)

    def guard(self, stopped=False, preparing=False):
        if rospy.is_shutdown():
            raise Stopped('ROS 已退出')
        why = self.s.issue or self.inputs_problem()
        if why:
            raise Stopped(why)
        if self.reference is not None:
            r, t = self.transform()
            if np.linalg.norm(r - self.reference[0]) > .01 or np.linalg.norm(t - self.reference[1]) > .01:
                self.s.stop('地图变换已重置，本轮作废')
                raise Stopped(self.s.issue)
        if self.real:
            f = self.s.fcu
            if f is None or time.monotonic() - self.s.fcu_at >= 2. or not f.connected:
                raise Stopped('飞控状态断流或连接断开')
            if not self.fc.ext_fresh() or self.fc.pos() is None or self.fc.descent() is None:
                raise Stopped('飞控着地/位置/速度状态断流')
            if not np.isfinite(self.fc.pos()).all() or not math.isfinite(self.fc.descent()):
                raise Stopped('飞控位置/速度数值无效')
            if self.s.ekf_problem():
                raise Stopped(self.s.ekf_problem())
            phase = self.s.phase
            allowed = {'PREFLIGHT': ('POSCTL',), 'WAIT_ARM': ('POSCTL',),
                       'COUNTDOWN': ('POSCTL',), 'OFFBOARD_ENTRY': ('POSCTL', 'OFFBOARD'),
                       'TAKEOFF': ('OFFBOARD',), 'ROUTE': ('OFFBOARD',),
                       'LAND_REQUEST': ('OFFBOARD',),
                       'LANDING': ('OFFBOARD', 'POSCTL') if not f.armed else ('OFFBOARD',)}[phase]
            if f.mode not in allowed:
                raise Stopped('飞手接管/模式改变：' + f.mode)
            if phase == 'OFFBOARD_ENTRY' and f.mode != 'OFFBOARD' and self.s.offboard_entered_at is not None:
                raise Stopped('切入 OFFBOARD 后又切出，本轮终止')
            if phase == 'ROUTE' and self.fc.landed != ExtendedState.LANDED_STATE_IN_AIR:
                raise Stopped('航线期间飞控不再报在空中')
            if phase in ('COUNTDOWN', 'OFFBOARD_ENTRY', 'TAKEOFF', 'ROUTE') and not f.armed:
                raise Stopped('飞控上锁')
            if phase == 'PREFLIGHT' and f.armed:
                raise Stopped('预检期间已解锁')
            if phase in ('PREFLIGHT', 'WAIT_ARM', 'COUNTDOWN', 'OFFBOARD_ENTRY') and not self.fc.on_ground():
                raise Stopped('地面预检/起飞准备期间不在地面')
            if phase in ('LAND_REQUEST', 'LANDING') and self.s.disarmed_at is not None and not self.fc.on_ground():
                if time.monotonic() - self.s.disarmed_at > 2.:
                    raise Stopped('已上锁但没有新鲜着地确认，不算成功')
        if self.s.phase == 'ROUTE' or not self.real:
            super().guard(stopped=stopped, preparing=preparing)
            self.trace_leg()
        elif self.s.phase == 'TAKEOFF' and self.s.get('gateway_status') != 'READY':
            raise Stopped('起飞网关不再 READY')
        if self.stage_deadline is not None and time.monotonic() >= self.stage_deadline:
            raise Stopped(self.s.phase + ' 阶段等待超时')
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise Stopped('单点任务超时（20 s）')
        if self.holding and self.current == 5:
            self.check_return()

    def check_return(self):
        p = self.s.pose()
        if p is None or np.linalg.norm(p[0] - self.goals[-1]) > .15 or p[2] > .1 or \
                abs(wrap(p[1] - self.yaws[-1])) > math.radians(5):
            raise Stopped('返回点位置/速度/航向未稳定，不降落')

    def hold_for(self, i):
        return HOLDS[i]

    def trace_leg(self):
        if self.current is None or not self.record or len(self.record['legs']) <= self.current:
            return
        leg = self.record['legs'][self.current]
        p = self.s.pose()
        if not p:
            return
        near = np.linalg.norm(p[0] - self.goals[self.current]) <= .15 and p[2] <= .1
        yaw_gap = abs(wrap(p[1] - self.yaws[self.current]))
        label = ('HOLD' if self.holding else 'TURN' if near and yaw_gap > math.radians(5) and
                 self.s.get('executor_status') == 'HOLD' else 'SETTLING' if near and
                 yaw_gap <= math.radians(5) else 'MOTION')
        events = leg.setdefault('phases', [])
        if not events or events[-1]['phase'] != label:
            events.append(dict(phase=label, wall=time.time(), ros=rospy.Time.now().to_sec(),
                               state=self.snapshot()))
            self.save()

    def pause(self, seconds, **guards):
        self.holding = bool(guards.get('stopped')) and self.current is not None
        try:
            return super().pause(seconds, **guards)
        finally:
            self.holding = False

    def send_leg(self, i, goal, goal_map, yaw_map):
        self.current = i
        self.deadline = time.monotonic() + LEG_TIMEOUT
        try:
            # Existing action/result/bounds/cancel/reprepare semantics stay unchanged.
            return super().send_leg(i, goal, goal_map, yaw_map)
        finally:
            self.deadline = None
            self.current = None

    def ground_geometry(self):
        p, a, speed = self.s.pose()
        r, t = self.reference
        here = r @ p + t
        if np.linalg.norm(here[:2]) > .30 or abs(here[2]) > .10:
            raise Stopped('起点不在 map 原点附近（水平≤0.30 m、高度±0.10 m），拒绝固定路线')
        if abs(wrap(heading(r, a))) > math.radians(10):
            raise Stopped('起飞机头偏离 map 0° 超过10°')
        if speed > .1:
            raise Stopped('地面未静止')
        return p

    def create_records(self):
        out = os.path.expanduser(self.a.out or '~/test_data/nav_test/' + time.strftime('%Y%m%d_%H%M%S') + '_test10')
        os.makedirs(out, exist_ok=True)
        suffix = '%s_%d' % (time.strftime('%H%M%S'), time.time_ns())
        self.record_path = os.path.join(out, 'goal_test10_' + suffix + '.json')
        self.flight_path = os.path.join(out, 'flight_test10_' + suffix + '.json')
        self.flight = dict(test='test10', map_id=self.cfg['map_id'], real_output=self.real,
                           phases=[], end='RUNNING', start_wall=time.time(), start_ros=rospy.Time.now().to_sec())
        self.record = dict(test='test10', map_id=self.cfg['map_id'], real_output=self.real,
                           trigger='manual_arm', goals_map=[list(p[:3]) for p in ROUTE],
                           goal_yaw_map_deg=[p[3] for p in ROUTE], legs=[], hold_s=list(HOLDS),
                           end='RUNNING', start_wall=time.time(), start_ros=rospy.Time.now().to_sec())
        self.save()

    def preflight(self):
        profile(self.cfg)
        self.guard()
        if self.s.initial_armed or self.s.fcu.armed or not self.fc.on_ground() or self.fc.ground_z is None:
            raise Stopped('必须从地面上锁状态启动，不自动上锁/解锁')
        self.reference = self.transform()  # Only capture ONCE, before any arming.
        p = self.ground_geometry()
        self.goals_map = np.array([p[:3] for p in ROUTE])
        r, t = self.reference
        yaw = self.s.pose()[1]
        if np.linalg.norm((r @ np.array([math.cos(yaw), math.sin(yaw), 0.]))[:2]) < .9:
            raise Stopped('地图变换不能保持水平航向')
        self.goals = (self.goals_map - t) @ r
        self.yaw_map = np.radians([p[3] for p in ROUTE])
        self.yaws = np.array([heading(r.T, a) for a in self.yaw_map])
        self.names = NAMES
        expected = p + np.array([0., 0., 1.])
        check_goal_bounds(np.vstack([expected, self.goals]), self.cfg,
                          ['预计起飞终点'] + list(NAMES))
        for target in np.vstack([expected, self.goals]):
            self.guard()
            reply = self.check(path_of([target]))
            if not reply.valid:
                raise Stopped('起飞终点/路线目标包络预检拒绝：' + getattr(reply, 'reason', 'unknown/occupied'))
        self.record.update(goals_odom=self.goals.tolist(), goal_yaw_odom_deg=np.degrees(self.yaws).tolist(),
                           transform_rotation=r.tolist(), transform_translation=t.tolist(),
                           max_vel=self.cfg['max_vel'], max_acc=self.cfg['max_acc'])
        self.flight.update(transform_rotation=r.tolist(), transform_translation=t.tolist(),
                           start_ground=self.snapshot(), goal_record=os.path.basename(self.record_path))
        self.save()

    def wait_arm(self):
        self.phase('WAIT_ARM')
        say('预检通过：通道6开录包，然后 POSCTL 地面人工解锁；只触发这一轮')
        while self.s.arm_at is None:
            self.guard()
            self.ground_geometry()
            time.sleep(.05)
        self.guard()
        self.ground_geometry()
        self.ground_z = float(self.fc.pos()[2])  # Freeze: Fc starts updating ground_z again after disarming.
        self.flight.update(arm_wall=time.time(), ground_z=self.ground_z)

    def trigger_takeoff(self):
        self.phase('COUNTDOWN')
        self.guard()
        self.stage_deadline = time.monotonic() + 1.
        result = None
        for attempt in range(3):
            self.guard()
            result = self.prepare_takeoff()
            if result.success:
                break
            if attempt < 2:
                time.sleep(.2)
        if not result.success:
            raise Stopped('prepare_takeoff 被拒：' + result.message)
        self.stage_deadline = None
        while time.monotonic() - self.s.arm_at < self.a.delay:
            self.guard()
            time.sleep(.05)
        self.phase('OFFBOARD_ENTRY')
        started, next_try, tries = time.monotonic(), time.monotonic(), 0
        self.stage_deadline = started + 4.5
        while not self.s.offboard() or self.s.get('gateway_status') != 'READY':
            self.guard()
            if time.monotonic() - started > 4.5:
                raise Stopped('切 OFFBOARD 超时')
            if tries < 3 and time.monotonic() >= next_try:
                self.guard()
                self.set_mode(base_mode=0, custom_mode='OFFBOARD')
                tries += 1
                next_try = time.monotonic() + 1.5
            time.sleep(.05)
        self.phase('TAKEOFF')
        self.guard()
        if not self.fc.on_ground():
            raise Stopped('发起飞请求前不在地面')
        start = time.monotonic()
        self.stage_deadline = start + TAKEOFF_TIMEOUT
        reply = self.takeoff()
        if not reply.success:
            raise Stopped('takeoff 被拒：' + reply.message)
        self.flight['takeoff_request_wall'] = time.time()
        stable_at = None
        while True:
            self.guard()
            now, p = time.monotonic(), self.s.pose()
            h = float(self.fc.pos()[2] - self.ground_z)
            map_z = float((self.reference[0] @ p[0] + self.reference[1])[2])
            ok = (self.s.get('gateway_status', 'takeoff') == 'DONE' and
                  self.fc.landed == ExtendedState.LANDED_STATE_IN_AIR and
                  abs(h - 1.) <= .10 and abs(map_z - 1.) <= .10 and p[2] <= .1)
            stable_at = (now if stable_at is None else stable_at) if ok else None
            if stable_at is not None and now - stable_at >= STABLE_SECONDS:
                check_goal_bounds([p[0]], self.cfg, ['实测起飞终点'])
                self.flight['takeoff_stable'] = self.snapshot()
                self.save()
                self.stage_deadline = None
                return
            if now - start >= TAKEOFF_TIMEOUT:
                raise Stopped('起飞高度/速度未连续稳定2 s（起飞20 s超时）')
            time.sleep(.05)

    def fly_route(self):
        self.phase('ROUTE')
        for i, (goal, gm, yaw) in enumerate(zip(self.goals, self.goals_map, self.yaw_map)):
            if i:
                self.reprepare()
            self.send_leg(i, goal, gm, yaw)
        self.guard(stopped=True)
        self.check_return()
        self.record['end'] = 'SUCCEEDED'
        self.record['end_wall'] = time.time()
        self.save()

    def land_once(self):
        self.guard(stopped=True)
        self.check_return()
        why = precheck(self.s, self.fc)
        if why:
            raise Stopped('降落预检拒绝：' + why)
        # Check the last action really succeeded; never clear an arbitrary fault.
        if len(self.record['legs']) != 6 or any(l['end'] != 'SUCCEEDED' for l in self.record['legs']):
            raise Stopped('路线没有全部成功，不自动降落')
        self.phase('LAND_REQUEST')
        start = time.monotonic()
        self.stage_deadline = start + LAND_TIMEOUT
        self.flight['land_request_wall'] = time.time()
        # The completed route leaves the gateway in its normal STOPPED fault, and a stopped gateway streams
        # nothing. Prepare it again (1 s to READY) and let /drone_nav/land descend inside OFFBOARD at ~0.1 m/s
        # to the ground reference taken at takeoff; PX4's own land detector then arms COM_DISARM_LAND.
        self.reprepare()
        self.guard()
        reply = self.land_start()
        if not reply.success:
            raise Stopped('慢速降落被拒：' + reply.message)
        self.s.land_requested = True
        self.flight['land_service_wall'] = time.time()
        while True:
            self.guard()
            now = time.monotonic()
            if self.s.land_confirmed_at is not None and self.s.phase != 'LANDING':
                self.flight['land_confirmed_wall'] = time.time()
                self.phase('LANDING')
            if self.s.land_confirmed_at is not None and self.fc.on_ground() and not self.s.fcu.armed:
                self.flight.update(touchdown_disarmed=self.snapshot(), landed_wall=time.time(),
                                   landing_s=now - start)
                self.stage_deadline = None
                return
            if now - start >= LAND_TIMEOUT:
                raise Stopped('慢速降落 %g s 超时（着地和上锁必须同时有新鲜确认）' % LAND_TIMEOUT)
            time.sleep(.05)

    def run_once(self):
        if self.used:
            raise Stopped('test10 已执行过一轮，禁止自动重跑')
        self.used = True
        self.create_records()
        self.s.begin(True, self.cfg['map_id'])
        try:
            self.phase('PREFLIGHT')
            self.preflight()
            self.wait_arm()
            self.trigger_takeoff()
            self.fly_route()
            self.land_once()
            self.flight['end'] = 'SUCCEEDED'
            self.phase('DONE')
            say('test10 完整成功：已确认着地且上锁。先停止通道6录包，再拿飞机，最后 Ctrl+C')
            return True
        except BaseException as e:
            cancel_error = ''
            if self.active:
                try:
                    self.client.cancel_goal()
                except Exception as failure:
                    cancel_error = '；取消请求异常：' + str(failure)
                self.active = False
            reason = 'Ctrl+C，本轮永久终止' if isinstance(e, KeyboardInterrupt) else str(e)
            reason += cancel_error
            if self.record['end'] == 'RUNNING':
                self.record.update(end='INTERRUPTED' if isinstance(e, KeyboardInterrupt) else 'ABORTED', reason=reason)
                if self.record['legs'] and self.record['legs'][-1]['end'] == 'RUNNING':
                    self.record['legs'][-1].update(end=self.record['end'], reason=reason)
            self.flight.update(end='INTERRUPTED' if isinstance(e, KeyboardInterrupt) else 'ABORTED', reason=reason,
                               stop_state=self.snapshot())
            say('test10 终止：%s；不会续飞/抢回模式。定位链路保留，请飞手接管或监视已开始的降落' % reason)
            if isinstance(e, KeyboardInterrupt):
                raise
            return False
        finally:
            self.s.end()
            self.deadline = self.stage_deadline = None
            self.flight.update(end_wall=time.time(), end_ros=rospy.Time.now().to_sec())
            self.save()
            say('完整流程记录：' + self.flight_path)


def main():
    a = arguments(rospy.myargv()[1:])
    if a.preview:
        # This path never even constructs takeoff or set_mode service proxies.
        from nav_global import serve
        a.points, a.hold, a.check_segments, a.max_start_offset = list(ROUTE), 0., False, 3.
        class PreviewRunner(GlobalRunner):
            test_name = record_prefix = 'test10'
            def __init__(self, args, cfg, *services):
                profile(cfg)
                super().__init__(args, cfg, *services)
            def hold_for(self, i):
                return HOLDS[i]
        say('test10 PREVIEW：仅预览空中六段，不验证起飞/降落，不切飞控模式')
        return serve('nav_test10_preview', a, PreviewRunner)
    rospy.init_node('nav_test10', anonymous=True, disable_signals=True)
    s = FullState()
    fc = Fc(s)
    s.fc = fc
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)  # noqa: F841
    end = time.monotonic() + 15.
    while s.fcu is None or not rospy.has_param('/drone_nav/config'):
        if rospy.is_shutdown() or time.monotonic() >= end:
            raise Stopped('15 s没收到配置/飞控状态')
        time.sleep(.05)
    if s.initial_armed:
        raise Stopped('启动时已经解锁，test10 不触发')
    for name in ('/drone_nav/prepare', '/drone_nav/check_trajectory',
                 '/drone_nav/prepare_takeoff', '/drone_nav/takeoff', '/drone_nav/land', '/mavros/set_mode'):
        rospy.wait_for_service(name, timeout=10)
    client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
    if not client.wait_for_server(rospy.Duration(10)):
        raise Stopped('10 s没等到 execute_route')
    runner = FullRunner(a, cfg, s, buf, rospy.ServiceProxy('/drone_nav/prepare', Trigger),
                        rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory), client, fc,
                        rospy.ServiceProxy('/drone_nav/prepare_takeoff', Trigger),
                        rospy.ServiceProxy('/drone_nav/takeoff', Trigger),
                        rospy.ServiceProxy('/drone_nav/land', Trigger),
                        rospy.ServiceProxy('/mavros/set_mode', SetMode))
    end = time.monotonic() + 15.
    while runner.inputs_problem() or not fc.ext_fresh() or fc.pos() is None or fc.descent() is None:
        if s.fcu.armed or rospy.is_shutdown() or time.monotonic() >= end:
            raise Stopped('输入未就绪/预检前已解锁，不起飞')
        time.sleep(.05)
    return 0 if runner.run_once() else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as e:
        say('test10 拒绝/终止：%s；请飞手确认模式，定位链路保留' % e)
        raise SystemExit(1)
