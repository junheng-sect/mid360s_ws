#!/usr/bin/env python3
"""EGO 单点测试：以当前位姿为基准，往机头方向前进 DIST 米（默认 1 m），保持当前高度和航向。

导航只收 drone_map 下的绝对目标：这里读 /drone_nav/odom（drone_odom 系、PX4 控制参考点），
在机体水平朝向上加偏移，再用重定位给的 drone_map<-drone_odom 换到地图系，经 /drone_nav/execute_route 发出。
目标只在发送那一刻按当时的位姿算一次，不随飞机移动。

两种触发方式：
  回车（默认；只预览时只能用这个）：prepare → 等网关 READY（真实输出要先切 OFFBOARD）→ 打印目标 → 回车发送，发一次就退出。
  --auto-offboard SEC（实飞）：不用碰键盘。没在 OFFBOARD 时一直保持 prepare（设定值跟着当前位置预发送）；
    遥控器每切进一次 OFFBOARD，原地保持 SEC 秒后自动发一次（这期间切出 OFFBOARD 就取消）。到点后原地保持，
    切出再切回触发下一次，方向一前一后交替（没到点就不换方向），飞机始终在起点附近。
    已经在 OFFBOARD 时启动不会触发，要先切出去。
Ctrl+C：取消正在飞的任务（网关减速后原地保持，仍在 OFFBOARD）并退出，之后人工切回定点。
每次的目标、起点和结果写到 <--out>/goal_<时分秒>.json（默认 ~/test_data/nav_test/<时间>/），供事后和录包对齐分析。
一般由 scripts/nav_testN.sh 调用，也可以单独跑（导航、重定位要已经起好）。

用法: nav_forward.py [--dist 1.0] [--left 0] [--up 0] [--auto-offboard 3] [--yes] [--out DIR]
"""
import argparse, json, math, os, sys, time
import numpy as np
import rospy, actionlib, tf2_ros
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mavros_msgs.msg import State as FcuState, EstimatorStatus
from traj_utils.srv import CheckTrajectory
from drone_navigation.msg import ExecuteRouteAction, ExecuteRouteGoal

ACTION_STATE = {2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED', 8: 'RECALLED'}
# 网关 / 执行器（drone_navigation）的英文原因（按前缀匹配）→ 现场看得懂的说明
HINTS = [
    ('alignment, inputs or exclusive ownership not ready',
     '网关还没对齐 LIO 和飞控本地位置（导航刚起要 2 s）、导航输入没就绪，或有别的节点在发 setpoint'),
    ('PX4 local estimate unavailable', '飞控本地位置不可用'),
    ('LIO/PX4 frame residual', 'LIO 和飞控本地位置差超过 15 cm 或 3°'),
    ('another setpoint publisher or ownership check lost', '有别的节点在发 setpoint'),
    ('navigation inputs/clock stale', '导航输入或时钟断了'),
    ('no valid trajectory for current goal within 3s: unknown footprint',
     '3 s 内没等到能执行的轨迹：轨迹经过的机体范围（body_half_extent）里有雷达没观测到的格子'),
    ('no valid trajectory', '3 s 内没等到能执行的轨迹'),
    ('trajectory no longer clear', '新观测显示轨迹不再安全'),
    ('planner heartbeat lost', 'EGO 心跳断了'),
    ('planner reported emergency', 'EGO 报紧急停止'),
    ('executor odometry stale', '执行器里程计断了'),
]


def hint(reason):
    for k, v in HINTS:
        if reason.startswith(k):
            return '%s（%s）' % (reason, v)
    return reason


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rot(q):
    return np.array([[1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                     [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)],
                     [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)]])


def path_of(points):
    p = Path()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = 'drone_odom'
    for x in points:
        q = PoseStamped()
        q.header = p.header
        q.pose.position.x, q.pose.position.y, q.pose.position.z = map(float, x)
        q.pose.orientation.w = 1.0
        p.poses.append(q)
    return p


def say(msg):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)


class State:
    odom_receive_timeout = 0.30

    def __init__(self):
        self.odom = None
        self.odom_at = 0.0
        self.st = {}
        self.fcu = None
        self.fcu_at = 0.0
        self.est = None
        self.est_at = 0.0
        rospy.Subscriber('/drone_nav/odom', Odometry, self.on_odom, queue_size=1)
        for name in ('input_status', 'gateway_status', 'executor_status', 'planner_status'):
            rospy.Subscriber('/drone_nav/' + name, String, self.on_status, callback_args=name, queue_size=5)
        rospy.Subscriber('/drone_reloc/status', String, self.on_status, callback_args='reloc', queue_size=5)
        self.result = {}
        rospy.Subscriber('/drone_reloc/result', String, self.on_result, queue_size=5)  # latched
        rospy.Subscriber('/mavros/state', FcuState, self.on_fcu, queue_size=5)
        rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, self.on_est, queue_size=5)

    def on_est(self, m):
        self.est, self.est_at = m, time.monotonic()

    def ekf_problem(self):
        """网关真实输出要的飞控 EKF 标志（和 gateway.estimator_ok 同一套），缺哪项说哪项；都满足返回空串。"""
        e = self.est
        if e is None or time.monotonic() - self.est_at > 2.0:
            return '飞控 /mavros/estimator_status 2 s 没更新'
        bad = [n for n, ok in (('姿态', e.attitude_status_flag), ('水平速度', e.velocity_horiz_status_flag),
                               ('垂直速度', e.velocity_vert_status_flag),
                               ('水平位置', e.pos_horiz_rel_status_flag or e.pos_horiz_abs_status_flag),
                               ('高度', e.pos_vert_abs_status_flag or e.pos_vert_agl_status_flag)) if not ok]
        why = ['%s无效' % '、'.join(bad)] if bad else []
        if e.const_pos_mode_status_flag:
            why.append('处于 const_pos（飞机静止在地面时 PX4 就报这个，起飞后消失）')
        return '飞控 EKF %s' % '，'.join(why) if why else ''

    def resting_only(self):
        """EKF 只差 const_pos（飞机静止）：在地面待命时网关不就绪是正常的。"""
        e = self.est
        return (e is not None and time.monotonic() - self.est_at <= 2.0 and e.const_pos_mode_status_flag
                and self.ekf_problem() == '飞控 EKF 处于 const_pos（飞机静止在地面时 PX4 就报这个，起飞后消失）')

    def on_result(self, m):
        try:
            self.result = json.loads(m.data)
        except ValueError:
            pass

    def on_odom(self, m):
        self.odom, self.odom_at = m, time.monotonic()

    def on_fcu(self, m):
        self.fcu, self.fcu_at = m, time.monotonic()

    def on_status(self, m, name):
        try:
            s = json.loads(m.data)
        except ValueError:
            s = None
        if not isinstance(s, dict):  # EGO 的 planner_status 是纯文本：状态号（"2" 会被解析成整数）或 "FAULT"
            s = {'state': m.data}
        s['_at'] = time.monotonic()
        self.st[name] = s

    def get(self, name, key='state'):
        s = self.st.get(name, {})
        return s.get(key, '') if time.monotonic() - s.get('_at', 0) < 1.5 else ''

    def offboard(self):
        """和网关同一个判据：已连接、已解锁、模式 OFFBOARD；状态 2 s 没更新按不在 OFFBOARD 算。"""
        m = self.fcu
        return m is not None and time.monotonic() - self.fcu_at < 2.0 and m.connected and m.armed and m.mode == 'OFFBOARD'

    def pose(self):
        o = self.odom
        if o is None or time.monotonic() - self.odom_at > self.odom_receive_timeout:
            return None
        p = o.pose.pose.position
        v = o.twist.twist.linear  # 机体系
        return np.array([p.x, p.y, p.z]), yaw_of(o.pose.pose.orientation), math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=1.0, help='沿机头水平方向前进多少米（负数后退）')
    ap.add_argument('--left', type=float, default=0.0, help='向左平移多少米（负数向右）')
    ap.add_argument('--up', type=float, default=0.0, help='高度变化多少米（默认 0，保持当前高度）')
    ap.add_argument('--auto-offboard', type=float, default=0.0, metavar='SEC',
                    help='实飞自动触发：每次切进 OFFBOARD 保持 SEC 秒后自动发，一前一后交替（0 = 回车触发）')
    ap.add_argument('--yes', action='store_true', help='回车模式下不等回车，目标算好直接发')
    ap.add_argument('--out', default='', help='目标记录目录（默认 ~/test_data/nav_test/<时间>）')
    a = ap.parse_args(rospy.myargv()[1:])
    off = math.hypot(a.dist, a.left)
    if off > 3.0 or abs(a.up) > 1.0:
        sys.exit('偏移太大（水平 %.2f m、高度 %.2f m）：这个脚本只做短距离测试，水平 ≤3 m、高度 ≤1 m' % (off, a.up))
    if not 0.0 <= a.auto_offboard <= 30.0:
        sys.exit('--auto-offboard 要在 0~30 s 之间')

    rospy.init_node('nav_forward', anonymous=True, disable_signals=True)
    while not rospy.has_param('/drone_nav/config'):
        print('等 navigation.launch 加载配置 /drone_nav/config ...', flush=True)
        time.sleep(1.0)
    cfg = rospy.get_param('/drone_nav/config')
    lo, hi = np.array(cfg['bounds_min'], float), np.array(cfg['bounds_max'], float)
    s = State()
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.wait_for_service('/drone_nav/prepare')
    prepare = rospy.ServiceProxy('/drone_nav/prepare', Trigger)
    check = rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory)
    client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
    client.wait_for_server()
    out = os.path.expanduser(a.out or '~/test_data/nav_test/' + time.strftime('%Y%m%d_%H%M%S'))

    def wait(cond, what):
        last = None
        while True:
            ok, msg = cond()
            if ok:
                return
            if msg != last:
                say('等%s：%s' % (what, msg))
                last = msg
            time.sleep(0.2)

    def inputs_ready():
        miss = []
        if s.get('input_status') != 'READY':
            miss.append('导航输入 %s %s' % (s.get('input_status') or '无消息', s.get('input_status', 'reason')))
        if s.get('reloc') != 'LOCALIZED' or s.get('reloc', 'valid') is not True:
            miss.append('重定位 %s' % (s.get('reloc') or '无消息'))
        elif s.result.get('confirmed') is not True or s.result.get('map') != cfg['map_id']:
            miss.append('重定位结果的地图 %s 和导航配置 map_id %s 不一致' % (s.result.get('map'), cfg['map_id']))
        if s.pose() is None:
            miss.append('/drone_nav/odom 断了')
        if not s.get('gateway_status'):
            miss.append('网关无消息')
        return not miss, '；'.join(miss)

    def make_goal(sign):
        """按此刻位姿算目标：(起点, 航向, 目标) 在 drone_odom，另附目标/起点的 drone_map 坐标和地图系航向。"""
        p = s.pose()
        if p is None:
            raise ValueError('/drone_nav/odom 断了')
        p0, a0, _ = p
        if np.any(p0 < lo) or np.any(p0 > hi):
            raise ValueError('飞机现在在 drone_odom (%.2f, %.2f, %.2f)，不在范围 %s~%s 里（高度要 ≥ %.1f m，先起飞悬停）' % (
                *p0, lo.tolist(), hi.tolist(), lo[2]))
        fwd = np.array([math.cos(a0), math.sin(a0), 0.0])
        left = np.array([-math.sin(a0), math.cos(a0), 0.0])
        g = p0 + sign * (a.dist * fwd + a.left * left + np.array([0.0, 0.0, a.up]))
        if np.any(g < lo) or np.any(g > hi):
            raise ValueError('目标 (%.2f, %.2f, %.2f) 超出范围 %s~%s（drone_odom 系）' % (*g, lo.tolist(), hi.tolist()))
        if not check(path_of([g])).valid:
            raise ValueError('目标处机体包络未知或被占（雷达还没看到那块，或离障碍太近）')
        tr = buf.lookup_transform('drone_map', 'drone_odom', rospy.Time(0), rospy.Duration(0.5)).transform
        r = rot(tr.rotation)
        t = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        return p0, a0, g, r @ g + t, a0 + math.atan2(r[1, 0], r[0, 0]), r @ p0 + t

    def show(goal):
        p0, a0, g, gm, am, p0m = goal
        say('当前（drone_odom）%.2f %.2f %.2f，航向 %.1f°；目标（drone_odom）%.2f %.2f %.2f' % (*p0, math.degrees(a0), *g))
        say('当前（drone_map）%.2f %.2f %.2f；目标（drone_map）%.2f %.2f %.2f，航向 %.1f°' % (*p0m, *gm, math.degrees(am)))
        try:  # world map（car_bridge 按 map_frame*.yaml 发的 map->drone_map），只用来显示
            tr = buf.lookup_transform('map', 'drone_map', rospy.Time(0), rospy.Duration(0.2)).transform
            r, t = rot(tr.rotation), np.array([tr.translation.x, tr.translation.y, tr.translation.z])
            say('当前（map）%.2f %.2f %.2f；目标（map）%.2f %.2f %.2f，航向 %.1f°' % (
                *(r @ p0m + t), *(r @ gm + t), math.degrees(am + math.atan2(r[1, 0], r[0, 0]))))
        except Exception:
            say('（没有 map->drone_map，不显示 world map 坐标）')

    def send(goal, sign, trigger):
        """发出并盯到结束；返回结束状态。Ctrl+C 时取消任务、写好记录再把 KeyboardInterrupt 抛出去。"""
        p0, a0, g, gm, am, _ = goal
        ps = PoseStamped()
        ps.header.frame_id = 'drone_map'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, gm)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(am / 2), math.cos(am / 2)
        os.makedirs(out, exist_ok=True)
        rec_path = os.path.join(out, 'goal_%s.json' % time.strftime('%H%M%S'))
        rec = dict(args=vars(a), trigger=trigger, sign=sign, map_id=cfg['map_id'], real_output=real,
                   max_vel=cfg['max_vel'], max_acc=cfg['max_acc'], start_odom=p0.tolist(), start_yaw_deg=math.degrees(a0),
                   goal_odom=g.tolist(), goal_map=gm.tolist(), goal_yaw_map_deg=math.degrees(am),
                   sent_wall=time.time(), sent_ros=rospy.Time.now().to_sec())
        client.send_goal(ExecuteRouteGoal(map_id=cfg['map_id'], waypoints=[ps]))
        t0 = time.monotonic()
        say('已发送：目标离当前 %.2f m' % np.linalg.norm(g - p0))
        interrupted = False
        try:
            while client.get_state() in (0, 1):
                time.sleep(0.5)
                p = s.pose()
                print('%5.1f s｜%s/%s｜离目标 %s｜速度 %s' % (
                    time.monotonic() - t0, s.get('executor_status') or '-', s.get('gateway_status') or '-',
                    '%.3f m' % np.linalg.norm(g - p[0]) if p else '?', '%.2f m/s' % p[2] if p else '?'), flush=True)
        except KeyboardInterrupt:
            interrupted = True
            client.cancel_goal()
            client.wait_for_result(rospy.Duration(3))
            say('已取消：网关减速后原地保持（仍在 OFFBOARD），请人工切回定点')
        res = client.get_result()
        end = ACTION_STATE.get(client.get_state(), str(client.get_state()))
        if end != 'SUCCEEDED':
            time.sleep(0.2)  # 让执行器 / 网关的 FAULT 原因先到，下面一起记下
        rec.update(end=end, reason=res.reason if res else '', elapsed_s=time.monotonic() - t0, end_wall=time.time(),
                   executor_reason=s.get('executor_status', 'reason'), gateway_reason=s.get('gateway_status', 'reason'))
        p = s.pose()
        if p:
            rec.update(end_odom=p[0].tolist(), end_error_m=float(np.linalg.norm(g - p[0])))
        with open(rec_path, 'w') as f:
            json.dump(rec, f, indent=1, ensure_ascii=False)
        say('结果 %s %s，用时 %.1f s%s；记录在 %s' % (end, rec['reason'], rec['elapsed_s'],
            '，终点误差 %.3f m' % rec['end_error_m'] if 'end_error_m' in rec else '', rec_path))
        if end != 'SUCCEEDED':  # 任务层只说 "navigation health lost"，真正原因在执行器 / 网关
            for who in ('executor', 'gateway'):
                if rec[who + '_reason']:
                    say('%s：%s' % ('执行器' if who == 'executor' else '网关', hint(rec[who + '_reason'])))
        if interrupted:
            raise KeyboardInterrupt
        return end

    wait(inputs_ready, '输入和重定位')
    real = s.get('gateway_status', 'real_output') is True
    say('地图 %s｜%s｜三维限速 %.2f m/s、%.2f m/s²' % (cfg['map_id'], '真实输出' if real else '只预览（不发飞控）',
                                                cfg['max_vel'], cfg['max_acc']))

    if a.auto_offboard > 0:
        if not real:
            sys.exit('只预览时网关不发飞控，切不进 OFFBOARD：预览用回车模式（不加 --auto-offboard）')
        auto(a, s, prepare, make_goal, show, send)
        return

    p0 = s.pose()[0]
    if np.any(p0 < lo) or np.any(p0 > hi):
        sys.exit('飞机现在在 drone_odom (%.2f, %.2f, %.2f)，不在目标范围 %s~%s 里：先起飞悬停（高度 ≥ %.1f m）再发' % (
            *p0, lo.tolist(), hi.tolist(), lo[2]))
    if s.get('gateway_status') != 'READY':
        for _ in range(25):
            r = prepare()
            if r.success:
                break
            time.sleep(0.2)
        say('prepare：%s' % r.message)
        time.sleep(0.3)  # 等网关按新状态再发一拍，别读到 prepare 之前的 FAULT
        if not r.success:
            sys.exit('prepare 没成功（对齐没完成、输入没就绪，或有别的节点在发 setpoint），看 navigation.launch 那个终端')
        if real:
            say('>>> 设定值已开始预发送（跟着当前位置走）。现在用遥控器切 OFFBOARD，飞机应原地不动 <<<')

    def gateway_ready():
        g = s.get('gateway_status')
        if g == 'FAULT':
            ekf = s.ekf_problem() if real else ''
            sys.exit('网关故障：%s%s（重跑本脚本会重新 prepare）' % (hint(s.get('gateway_status', 'reason')), '｜' + ekf if ekf else ''))
        return g == 'READY', g or '无消息'

    wait(gateway_ready, '网关 READY（真实输出要已解锁并在 OFFBOARD）')
    try:
        goal = make_goal(1)
    except Exception as e:
        sys.exit('目标不可用：%s' % e)
    show(goal)
    if not a.yes:
        input('回车发送（发送时按那一刻的位置重新算目标），Ctrl+C 放弃 ')
        if s.get('gateway_status') != 'READY':
            sys.exit('网关已不是 READY（%s），没有发送' % (s.get('gateway_status') or '无消息'))
        try:
            goal = make_goal(1)
        except Exception as e:
            sys.exit('目标不可用：%s' % e)
    send(goal, 1, 'enter')
    if real:
        say('任务结束后网关原地保持（仍在 OFFBOARD）；再发一次会重新 prepare，或人工切回定点降落')


def auto(a, s, prepare, make_goal, show, send):
    """每切进一次 OFFBOARD，保持 a.auto_offboard 秒后发一次；到点后换方向。"""
    sign = 1
    way = lambda k: '前进 %.2f m' % a.dist if k > 0 else '往回 %.2f m' % a.dist
    say('自动模式：每次切进 OFFBOARD，原地保持 %.0f s 后自动%s；到点后切出再切回往回飞，一前一后交替。Ctrl+C 停止' % (
        a.auto_offboard, way(1)))
    if s.offboard():  # 只认「切进 OFFBOARD」这个动作：中途重启时飞机正在 OFFBOARD 里，不能直接倒计时
        say('现在已经在 OFFBOARD：这次不触发，先切回定点再切回来')
        while s.offboard():
            time.sleep(0.1)
        say('已切出 OFFBOARD')
    while True:
        # 没在 OFFBOARD：一直保持 prepare（设定值跟着当前位置预发送，切进来时飞机原地不动）。
        # prepare 成功后网关仍可能马上转 FAULT（比如飞控 EKF 还没用上外部定位），所以没就绪时要把网关原因和 EKF 标志一起报出来
        last_try, last_note = time.monotonic() - 0.7, None  # 刚切出时先等网关看到模式变化
        while not s.offboard():
            g = s.get('gateway_status')
            if g in ('PREPARED', 'READY'):
                note = '已就绪：设定值预发送中。起飞悬停后用遥控器切 OFFBOARD，%.0f s 后自动%s' % (a.auto_offboard, way(sign))
            elif time.monotonic() - last_try > 1.0:
                last_try = time.monotonic()
                try:
                    r = prepare()
                    why = '' if r.success else 'prepare 被拒：%s' % hint(r.message)
                except Exception as e:
                    why = 'prepare 调用失败：%s' % e
                time.sleep(0.3)  # 等网关按 prepare 后的状态再发一拍，再判断就绪没有
                g = s.get('gateway_status')
                if g in ('PREPARED', 'READY'):
                    continue
                if g == 'FAULT' and s.get('gateway_status', 'reason') == 'PX4 local estimate unavailable' and s.resting_only():
                    note = '地面待命：飞机静止时 PX4 报 const_pos，网关不就绪是正常的；起飞悬停后会自动变成「已就绪」'
                else:
                    parts = ['网关 %s' % (g or '无消息')]
                    if g == 'FAULT':
                        parts[0] += '：' + hint(s.get('gateway_status', 'reason'))
                    note = '还没就绪（每秒重试 prepare）｜%s' % '｜'.join(parts + [x for x in (why, s.ekf_problem()) if x])
            else:
                note = last_note
            if note and note != last_note:
                say(note)
            last_note = note
            time.sleep(0.1)
        say('进入 OFFBOARD：原地保持 %.0f s 后%s（这期间切出 OFFBOARD 就取消）' % (a.auto_offboard, way(sign)))
        t = time.monotonic()
        while time.monotonic() - t < a.auto_offboard and s.offboard():
            time.sleep(0.05)
        if not s.offboard():
            say('%.0f s 内切出了 OFFBOARD，没有发' % a.auto_offboard)
            continue
        try:
            if s.get('gateway_status') != 'READY':
                raise ValueError('｜'.join(x for x in ('网关 %s %s' % (s.get('gateway_status') or '无消息', hint(s.get('gateway_status', 'reason'))),
                                                      s.ekf_problem()) if x))
            goal = make_goal(sign)
        except Exception as e:
            say('没有发：%s。切回定点再切 OFFBOARD 重试' % e)
        else:
            show(goal)
            end = send(goal, sign, 'offboard+%.0fs' % a.auto_offboard)
            if end == 'SUCCEEDED':
                sign = -sign
                say('到点，原地保持。下一次：切回定点再切 OFFBOARD，%.0f s 后%s；不测了就切回定点降落' % (a.auto_offboard, way(sign)))
            else:
                say('没到点（%s），方向不变。切回定点再切 OFFBOARD 重试' % end)
        while s.offboard():
            time.sleep(0.1)
        say('已切出 OFFBOARD')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n停止（没有在飞的任务了）', flush=True)
        sys.exit(130)
