#!/usr/bin/env python3
"""降落测试：遥控器每切进一次 OFFBOARD，原地悬停 HOVER 秒后切到 PX4 降落模式（AUTO.LAND），一直盯到落地、自动上锁。

降落交给 PX4 自己的 AUTO.LAND：在切模式那一刻的水平位置原地下降（MPC_LAND_SPEED），着地检测后 COM_DISARM_LAND 秒自动上锁。
悬停期间是导航网关在 OFFBOARD 里保持原地；没在 OFFBOARD 时一直保持 prepare（设定值跟着当前位置预发送，切进来时飞机原地不动）。
悬停期间切出 OFFBOARD 就取消；已经在 OFFBOARD 时启动不会触发，要先切出去。
降落中动摇杆即切回定点（COM_RC_OVERRIDE=3）：这里只记录接管，不再切模式。
Ctrl+C：只停本脚本，不动飞控模式；已经在降落的话 PX4 继续降落。
每次的记录写到 <--out>/land_<时分秒>.json（悬停点、切模式耗时、下降速度、着地 / 上锁时刻、着地点离悬停点的水平偏移），供事后和录包对齐分析。
一般由 scripts/nav_test3.sh 调用，也可以单独跑（导航、重定位要已经起好）。

用法: nav_land.py [--hover 3] [--out DIR]
"""
import argparse, json, math, os, sys, time
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import SetMode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_forward import State, hint, say  # noqa: E402  同目录：导航 / 飞控状态订阅、网关原因说明

LANDED = {0: '未知', 1: '在地面', 2: '在空中', 3: '起飞中', 4: '降落中'}


class Fc:
    """飞控本地位置（px4_map）、竖直速度和着地状态；在地面且上锁时一直记地面高度，用来算离地高度。"""

    def __init__(self, s):
        self.s = s
        self.p, self.p_at = None, 0.0
        self.vz, self.v_at = 0.0, 0.0
        self.landed, self.landed_at = None, 0.0
        self.ground_z = None
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.on_odom, queue_size=5)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.on_vel, queue_size=5)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self.on_ext, queue_size=5)

    def on_odom(self, m):
        q = m.pose.pose.position
        self.p, self.p_at = np.array([q.x, q.y, q.z]), time.monotonic()
        # 只在上锁、没在升降时记：起飞时着地状态要等飞机升起来一段才变，解锁状态又只有 1 Hz，否则会把地面记高
        f, v = self.s.fcu, self.descent()
        if self.on_ground() and f is not None and not f.armed and time.monotonic() - self.s.fcu_at < 2.0 \
                and v is not None and abs(v) < 0.1:
            self.ground_z = q.z

    def on_vel(self, m):
        self.vz, self.v_at = m.twist.linear.z, time.monotonic()

    def on_ext(self, m):
        self.landed, self.landed_at = m.landed_state, time.monotonic()

    def ext_fresh(self):
        return time.monotonic() - self.landed_at < 1.0

    def on_ground(self):
        return self.ext_fresh() and self.landed == ExtendedState.LANDED_STATE_ON_GROUND

    def landed_name(self):
        return LANDED.get(self.landed, str(self.landed)) if self.ext_fresh() else '无消息'

    def pos(self):
        return self.p if self.p is not None and time.monotonic() - self.p_at < 0.5 else None

    def agl(self):
        p = self.pos()
        return None if p is None or self.ground_z is None else float(p[2] - self.ground_z)

    def descent(self):
        return -self.vz if time.monotonic() - self.v_at < 0.5 else None


def fmt_h(h):
    return '%.2f m' % h if h is not None else '?（启动时不在地面，没记到地面高度）'


def drift(p0, p):
    return float(np.linalg.norm(p[:2] - p0[:2])) if p is not None else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hover', type=float, default=3.0, help='切进 OFFBOARD 后原地悬停多少秒再降落（默认 3）')
    ap.add_argument('--out', default='', help='记录目录（默认 ~/test_data/nav_test/<时间>）')
    a = ap.parse_args(rospy.myargv()[1:])
    if not 0.0 < a.hover <= 30.0:
        sys.exit('--hover 要在 0~30 s 之间（不含 0）')

    rospy.init_node('nav_land', anonymous=True, disable_signals=True)
    s = State()
    fc = Fc(s)
    rospy.wait_for_service('/drone_nav/prepare')
    prepare = rospy.ServiceProxy('/drone_nav/prepare', Trigger)
    rospy.wait_for_service('/mavros/set_mode')
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    out = os.path.expanduser(a.out or '~/test_data/nav_test/' + time.strftime('%Y%m%d_%H%M%S'))

    # /mavros/state 只有 1 Hz：没收到就判断「是否已在 OFFBOARD」会当成不在，收到第一条就直接开始倒计时
    t = time.monotonic()
    while not s.get('gateway_status') or s.fcu is None:
        if time.monotonic() - t > 10:
            sys.exit('10 s 没收到%s：%s' % (('/drone_nav/gateway_status', '导航没起来？') if not s.get('gateway_status')
                                          else ('/mavros/state', '常驻 MAVROS 没连上飞控？')))
        time.sleep(0.2)
    if s.get('gateway_status', 'real_output') is not True:
        sys.exit('网关是只预览（不发飞控），切不进 OFFBOARD，没法测降落')

    say('降落测试：每次切进 OFFBOARD，原地悬停 %.0f s 后切 PX4 降落模式（AUTO.LAND），着地后飞控自动上锁。Ctrl+C 停止' % a.hover)
    if s.offboard():  # 只认「切进 OFFBOARD」这个动作：中途重启时飞机正在 OFFBOARD 里，不能直接倒计时
        say('现在已经在 OFFBOARD：这次不触发，先切回定点再切回来')
        while s.offboard():
            time.sleep(0.1)
        say('已切出 OFFBOARD')
    while True:
        wait_entry(a, s, prepare)
        t_entry = time.time()
        say('进入 OFFBOARD：原地悬停 %.0f s 后降落（这期间切出 OFFBOARD 就取消）' % a.hover)
        t = time.monotonic()
        while time.monotonic() - t < a.hover and s.offboard():
            time.sleep(0.05)
        if not s.offboard():
            say('%.0f s 内切出了 OFFBOARD，没有降落' % a.hover)
            continue
        why = precheck(s, fc)
        if why:
            say('没有降落：%s。飞机仍在 OFFBOARD 原地悬停：切回定点再切 OFFBOARD 重试，或手动降落' % why)
        else:
            end = land(a, s, fc, set_mode, out, t_entry)
            if end == 'DISARMED':
                say('已落地上锁。通道 6 拨回停止录包。再测：模式开关拨回定点，解锁起飞到约 1 m 悬停稳，再切 OFFBOARD；不测了直接 Ctrl+C 收尾')
            elif end == 'TAKEOVER':
                say('飞手接管。再测：悬停稳后切回 OFFBOARD（重新悬停 %.0f s 再降落）；不测了手动降落，上锁后 Ctrl+C' % a.hover)
        while s.offboard():  # 切模式失败或检查没过时飞机还在 OFFBOARD：等飞手切出去，下一次要重新切进来才触发
            time.sleep(0.1)


def wait_entry(a, s, prepare):
    """没在 OFFBOARD：一直保持 prepare，直到遥控器切进 OFFBOARD。和 nav_forward 的自动模式同一套提示。"""
    last_try, last_note = time.monotonic() - 0.7, None  # 刚切出时先等网关看到模式变化
    while not s.offboard():
        g = s.get('gateway_status')
        if g in ('PREPARED', 'READY'):
            note = '已就绪：设定值预发送中。起飞悬停后用遥控器切 OFFBOARD，%.0f s 后自动降落' % a.hover
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


def precheck(s, fc):
    """切降落前：飞控认为在空中、本地位置在更新、EKF 标志齐全（AUTO.LAND 靠它原地保持水平位置）。没问题返回空串。"""
    if not fc.ext_fresh():
        return '飞控 /mavros/extended_state 1 s 没更新，不知道是不是在空中'
    if fc.landed != ExtendedState.LANDED_STATE_IN_AIR:
        return '飞控着地状态是「%s」，不是在空中' % fc.landed_name()
    if fc.pos() is None:
        return '飞控本地位置 /mavros/local_position/odom 断了'
    return s.ekf_problem()


def land(a, s, fc, set_mode, out, t_entry):
    """切 AUTO.LAND 并盯到上锁或被接管；返回结束状态。Ctrl+C 时写好记录再把 KeyboardInterrupt 抛出去。"""
    def mode():  # MAVROS 的 /mavros/state 跟着飞控心跳 1 Hz 更新；2 s 没更新算不知道
        return s.fcu.mode if s.fcu is not None and time.monotonic() - s.fcu_at < 2.0 else ''

    def disarmed():
        return s.fcu is not None and time.monotonic() - s.fcu_at < 2.0 and not s.fcu.armed

    p0, h0, o = fc.pos(), fc.agl(), s.pose()
    os.makedirs(out, exist_ok=True)
    rec_path = os.path.join(out, 'land_%s.json' % time.strftime('%H%M%S'))
    rec = dict(args=vars(a), trigger='offboard+%gs' % a.hover, offboard_entry_wall=t_entry, request_wall=time.time(),
               gateway_at_request=s.get('gateway_status'), start_fc=p0.tolist(), start_agl=h0, ground_z=fc.ground_z,
               start_odom=o[0].tolist() if o else None, start_yaw_deg=math.degrees(o[1]) if o else None)
    say('悬停点（飞控本地）%.2f %.2f %.2f，离地 %s：切 AUTO.LAND' % (*p0, fmt_h(h0)))

    # 一个循环里既重试切模式（每次等 2.5 s，最多 3 次），又从发出请求起就盯高度和速度：
    # 模式要等下一拍 1 Hz 的 /mavros/state 才看得到，这段时间飞机其实已经在降了。以下时间都从发出请求算起
    t0 = time.monotonic()
    confirmed, tries, next_try = None, [], t0
    touch, vmax, last, warned, end = None, 0.0, 0.0, False, None
    try:
        while end is None:
            now, p, h, v = time.monotonic(), fc.pos(), fc.agl(), fc.descent()
            if v is not None:
                vmax = max(vmax, v)
            if 'contact_s' not in rec and h is not None and h <= 0.05:  # 离地 5 cm 内：估计的触地时刻（着地判定要再晚约 1 s）
                rec.update(contact_wall=time.time(), contact_s=now - t0)
            if touch is None and fc.on_ground():
                touch = now
                rec.update(touchdown_wall=time.time(), descent_s=now - t0,
                           touchdown_fc=p.tolist() if p is not None else None, touchdown_drift_m=drift(p0, p))
                say('飞控判定着地（请求降落后 %.1f s），着地点离悬停点水平 %s；等自动上锁（COM_DISARM_LAND）' % (
                    now - t0, '%.3f m' % drift(p0, p) if p is not None else '?'))
            if not confirmed and mode() == 'AUTO.LAND':
                confirmed = now
                rec.update(land_confirmed_wall=time.time(), mode_switch_s=now - t0)
                say('已进入 AUTO.LAND（请求后 %.1f s 看到，含 1 Hz 状态的延迟）。降落中动摇杆即切回定点接管' % (now - t0))
            if disarmed():
                end = 'DISARMED' if confirmed else 'TAKEOVER'
            elif mode() not in ('', 'OFFBOARD', 'AUTO.LAND') or (confirmed and mode() == 'OFFBOARD'):
                end = 'TAKEOVER'
            elif not confirmed and now >= next_try:
                if len(tries) == 3:
                    end = 'SET_MODE_FAILED'
                else:
                    try:
                        tries.append('已发出' if set_mode(base_mode=0, custom_mode='AUTO.LAND').mode_sent else 'MAVROS 没发出去')
                    except rospy.ServiceException as e:
                        tries.append('服务调用失败：%s' % e)
                    next_try = now + 2.5
            elif confirmed and now - last >= 0.5:
                last = now
                print('%5.1f s｜%s｜%s｜离地 %s｜下降 %s｜水平偏离 %s' % (
                    now - t0, mode() or '?', fc.landed_name(), '%.2f m' % h if h is not None else '?',
                    '%.2f m/s' % v if v is not None else '?', '%.3f m' % drift(p0, p) if p is not None else '?'), flush=True)
            if touch and not warned and now - touch > 10:
                warned = True
                say('着地 10 s 还没自动上锁：检查 COM_DISARM_LAND，或手动上锁')
            time.sleep(0.05)
    except KeyboardInterrupt:
        end = 'INTERRUPTED'
    rec.update(set_mode_tries=tries, max_descent_mps=vmax, end=end, end_mode=mode(), end_wall=time.time(), total_s=time.monotonic() - t0)
    if end == 'DISARMED' and touch:
        rec['disarm_after_touchdown_s'] = time.monotonic() - touch
    p = fc.pos()
    rec.update(end_fc=p.tolist() if p is not None else None, end_drift_m=drift(p0, p))
    with open(rec_path, 'w') as f:
        json.dump(rec, f, indent=1, ensure_ascii=False)
    if end == 'DISARMED':
        say('结果 DISARMED：请求降落到着地判定 %s，着地到上锁 %s，最大下降速度 %.2f m/s，着地点水平偏离 %s；记录在 %s' % (
            '%.1f s' % rec['descent_s'] if touch else '?（没收到着地状态）',
            '%.1f s' % rec['disarm_after_touchdown_s'] if touch else '?', vmax,
            '%.3f m' % rec['touchdown_drift_m'] if rec.get('touchdown_drift_m') is not None else '?', rec_path))
    elif end == 'SET_MODE_FAILED':
        say('切 AUTO.LAND 没成功（%s，每次等 2.5 s 模式都没变）。飞机仍在 OFFBOARD 原地悬停：请手动降落｜记录在 %s' % (
            '，'.join(tries), rec_path))
    elif end == 'TAKEOVER':
        say('%s模式变成 %s%s：飞手接管，不再干预｜记录在 %s' % (
            '' if confirmed else '还没看到 AUTO.LAND 就', mode() or '?', '、已上锁' if disarmed() else '', rec_path))
    else:  # INTERRUPTED
        say('停止盯降落（没动飞控模式，已发出降落请求的话 PX4 继续降落）｜记录在 %s' % rec_path)
        raise KeyboardInterrupt
    return end


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n停止', flush=True)
        sys.exit(130)
