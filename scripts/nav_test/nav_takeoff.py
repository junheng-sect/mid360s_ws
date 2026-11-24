#!/usr/bin/env python3
"""起飞测试（test6）：地面定点模式解锁后 DELAY 秒自动起飞到 takeoff_height（导航配置，默认 1 m）并悬停。

PX4 自带的 AUTO.TAKEOFF 用不了：EKF 用外部航向时 yaw_align 一直为 false，飞控没有全局位置，起飞完要切的 LOITER 不可用。
所以起飞也走导航网关的 OFFBOARD：
  1) 看到「上锁 → 解锁」（只认运行中看到的解锁动作），马上调 /drone_nav/prepare_takeoff：网关在地面预发送设定值，
     压在飞机下方 0.3 m，PX4 判定不要起飞，电机怠速
  2) 倒计时 DELAY 秒（期间上锁、拨模式就取消），脚本经 MAVROS 切 OFFBOARD
  3) 网关 READY 后调 /drone_nav/takeoff：从地面按 max_vel_z / max_acc_z 平滑爬升 takeoff_height，到高后网关保持悬停；
     PX4 自己做起转和推力斜坡（MPC_TKO_RAMP_T），离地后着地状态变成在空中
之后脚本只监视、不再切模式：飞手推杆即切回定点（COM_RC_OVERRIDE=3），或拨模式开关。
PX4 解锁后 COM_DISARM_PRFLT（10 s）内不起飞会自动上锁，所以 DELAY 最多 4 s。
记录写到 <--out>/takeoff_<时分秒>.json。一般由 scripts/nav_test6.sh 调用。

用法: nav_takeoff.py [--delay 3] [--out DIR]
"""
import argparse, json, os, sys, time
import numpy as np
import rospy
from std_srvs.srv import Trigger
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import SetMode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_forward import State, hint, say  # noqa: E402
from nav_land import Fc, drift  # noqa: E402  飞控本地位置、着地状态、上锁时记的地面高度


def armed(s):
    return s.fcu is not None and time.monotonic() - s.fcu_at < 2.0 and s.fcu.armed


def mode(s):  # /mavros/state 跟飞控心跳 1 Hz；2 s 没更新算不知道
    return s.fcu.mode if s.fcu is not None and time.monotonic() - s.fcu_at < 2.0 else ''


def call(srv):
    try:
        r = srv()
        return r.success, r.message
    except Exception as e:  # rospy.ServiceException 或连接断开
        return False, '服务调用失败：%s' % e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delay', type=float, default=3.0, help='解锁后多少秒起飞（默认 3，1~4）')
    ap.add_argument('--out', default='', help='记录目录（默认 ~/test_data/nav_test/<时间>）')
    a = ap.parse_args(rospy.myargv()[1:])
    if not 1.0 <= a.delay <= 4.0:
        sys.exit('--delay 要在 1~4 s：PX4 解锁后 10 s 不起飞会自动上锁')

    rospy.init_node('nav_takeoff', anonymous=True, disable_signals=True)
    s = State()
    fc = Fc(s)
    for name in ('/drone_nav/prepare_takeoff', '/drone_nav/takeoff', '/mavros/set_mode'):
        try:
            rospy.wait_for_service(name, timeout=10)
        except rospy.ROSException:
            sys.exit('10 s 没等到服务 %s：%s' % (name, '导航网关是旧版本？（要有地面起飞）' if 'drone_nav' in name else '常驻 MAVROS 没起来？'))
    prepare = rospy.ServiceProxy('/drone_nav/prepare_takeoff', Trigger)
    takeoff = rospy.ServiceProxy('/drone_nav/takeoff', Trigger)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    c = rospy.get_param('/drone_nav/config')
    h, vz, az = float(c.get('takeoff_height', 1.0)), float(c.get('max_vel_z', 0.3)), float(c.get('max_acc_z', 0.3))
    out = os.path.expanduser(a.out or '~/test_data/nav_test/' + time.strftime('%Y%m%d_%H%M%S'))

    t = time.monotonic()
    while not s.get('gateway_status') or s.fcu is None:
        if time.monotonic() - t > 10:
            sys.exit('10 s 没收到%s' % ('/drone_nav/gateway_status：导航没起来？' if not s.get('gateway_status')
                                      else '/mavros/state：常驻 MAVROS 没连上飞控？'))
        time.sleep(0.2)
    if s.get('gateway_status', 'real_output') is not True:
        sys.exit('网关是只预览（不发飞控），切不进 OFFBOARD，没法测起飞')

    say('起飞测试：定点模式解锁后 %.0f s 自动切 OFFBOARD，爬升 %.2f m（%.2f m/s、%.2f m/s²）后悬停。Ctrl+C 停止' % (a.delay, h, vz, az))
    while True:
        if armed(s):
            say('现在已经解锁：这次不触发。先上锁（落地），再在地面解锁才会起飞')
            while armed(s):
                time.sleep(0.1)
        wait_arm(s, fc)
        fly(a, s, fc, prepare, takeoff, set_mode, h, out)
        say('再测：落地上锁后重新解锁；不测了上锁后 Ctrl+C 收尾')


def wait_arm(s, fc):
    """等飞手在地面解锁；期间把影响起飞的问题说出来（变化时才打印）。"""
    last = None
    web_task = bool(os.environ.get('WAREHOUSE_TASK_ID'))
    ready = False
    while not armed(s):
        why = []
        if not fc.ext_fresh():
            why.append('没收到 /mavros/extended_state')
        elif fc.landed != ExtendedState.LANDED_STATE_ON_GROUND:
            why.append('飞控着地状态是「%s」' % fc.landed_name())
        if fc.ground_z is None:
            why.append('还没记到地面高度（飞机要静止在地面、上锁）')
        if s.get('input_status') != 'READY':
            why.append('导航输入 %s' % (s.get('input_status') or '无消息'))
        if web_task:
            if not s.fcu or not s.fcu.connected or time.monotonic() - s.fcu_at >= 2. or mode(s) != 'POSCTL':
                why.append('等待飞控连接并切到定点模式')
            if s.ekf_problem() and not s.resting_only():
                why.append(s.ekf_problem())
        ready = not why
        note = '待命：定点模式在地面解锁，%s' % ('；'.join(why) if why else '解锁后自动起飞')
        if note != last:
            say(note)
            last = note
        time.sleep(0.1)
    if web_task and not ready:
        say('预检完成前已解锁，本轮取消。上锁后重新下发')
        sys.exit(2)


def fly(a, s, fc, prepare, takeoff, set_mode, h, out):
    """解锁之后的一轮：预发送、倒计时、切 OFFBOARD、起飞、盯到悬停或被接管。每种结束都写记录。"""
    t_arm = time.monotonic()
    rec = dict(args=vars(a), takeoff_height=h, arm_wall=time.time(), arm_mode=mode(s), ground_z=fc.ground_z,
               start_fc=fc.pos().tolist() if fc.pos() is not None else None)
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, 'takeoff_%s.json' % time.strftime('%H%M%S'))

    def stop(end, msg):
        rec.update(end=end, end_mode=mode(s), end_wall=time.time())
        with open(path, 'w') as f:
            json.dump(rec, f, indent=1, ensure_ascii=False)
        say('%s｜记录在 %s' % (msg, path))
        return rec

    if mode(s) != 'POSCTL':
        return stop('NOT_POSCTL', '解锁时模式是 %s，不是定点：不起飞（10 s 后飞控自动上锁）' % (mode(s) or '?'))
    if not fc.on_ground() or fc.ground_z is None:
        return stop('NOT_ON_GROUND', '飞控不报在地面或没记到地面高度：不起飞')
    ok, msg = False, ''
    while time.monotonic() - t_arm < 1.0 and not ok:
        ok, msg = call(prepare)
        if not ok:
            time.sleep(0.2)
    if not ok:
        return stop('PREPARE_FAILED', 'prepare_takeoff 被拒：%s。不起飞（10 s 后飞控自动上锁）' % hint(msg))
    say('已解锁、网关在地面预发送。%.0f s 后切 OFFBOARD 起飞；现在上锁或拨模式开关即取消' % a.delay)
    while time.monotonic() - t_arm < a.delay:
        if not armed(s) or mode(s) not in ('', 'POSCTL') or s.get('gateway_status') == 'FAULT':
            return stop('CANCELLED', '倒计时中%s：取消起飞' % ('上锁了' if not armed(s) else '网关故障（%s）' % hint(
                s.get('gateway_status', 'reason')) if s.get('gateway_status') == 'FAULT' else '模式变成 %s' % mode(s)))
        time.sleep(0.05)

    # 切 OFFBOARD：模式要等 1 Hz 的 /mavros/state 才看得到，网关 READY 同理；最多试 3 次、每次等 1.5 s
    t0, tries = time.monotonic(), []
    while s.get('gateway_status') != 'READY':
        if not armed(s):
            return stop('DISARMED', '切 OFFBOARD 前飞控上锁了：不起飞')
        if s.get('gateway_status') == 'FAULT':
            return stop('GATEWAY_FAULT', '网关故障：%s。不起飞' % hint(s.get('gateway_status', 'reason')))
        if mode(s) not in ('', 'POSCTL', 'OFFBOARD'):
            return stop('TAKEOVER', '模式变成 %s：飞手接管，不起飞' % mode(s))
        if len(tries) < 3 and time.monotonic() - t0 >= 1.5 * len(tries):
            try:
                tries.append('已发出' if set_mode(base_mode=0, custom_mode='OFFBOARD').mode_sent else 'MAVROS 没发出去')
            except rospy.ServiceException as e:
                tries.append('服务调用失败：%s' % e)
        if time.monotonic() - t0 > 4.5:
            return stop('OFFBOARD_FAILED', '切 OFFBOARD 没成功（%s）：不起飞，飞机还在地面（10 s 后飞控自动上锁）' % '，'.join(tries))
        time.sleep(0.05)
    rec.update(offboard_s=time.monotonic() - t_arm, set_mode_tries=tries)
    ok, msg = call(takeoff)
    if not ok:
        return stop('TAKEOFF_REFUSED', '网关拒绝起飞：%s。飞机在 OFFBOARD 地面怠速：拨回定点上锁' % msg)
    t_climb = time.monotonic()
    rec.update(takeoff_request_wall=time.time(), takeoff_after_arm_s=t_climb - t_arm)
    say('开始爬升 %.2f m（解锁后 %.1f s）。推杆即切回定点接管' % (h, t_climb - t_arm))
    return watch(s, fc, h, rec, t_climb, stop)


def watch(s, fc, h, rec, t0, stop):
    """盯爬升和悬停：离地、到高、超调、悬停误差；切出 OFFBOARD、上锁或 Ctrl+C 结束。"""
    p0 = fc.pos()
    hmax, vmax, last, hover = 0.0, 0.0, 0.0, []
    try:
        while True:
            now, p, agl, v = time.monotonic(), fc.pos(), fc.agl(), fc.descent()
            st = s.get('gateway_status', 'takeoff')
            if agl is not None:
                hmax = max(hmax, agl)
            if v is not None:
                vmax = max(vmax, -v)
            if 'liftoff_s' not in rec and fc.ext_fresh() and fc.landed != ExtendedState.LANDED_STATE_ON_GROUND:
                rec['liftoff_s'] = now - t0
                say('飞控判定离地（开始爬升后 %.1f s，%s）' % (now - t0, fc.landed_name()))
            if 'reach_s' not in rec and agl is not None and abs(agl - h) <= 0.05:
                rec['reach_s'] = now - t0
            if st == 'DONE' and 'done_s' not in rec:
                rec['done_s'] = now - t0
                say('到高悬停：离地 %s，目标 %.2f m（开始爬升后 %.1f s）' % ('%.2f m' % agl if agl is not None else '?', h, now - t0))
            if 'done_s' in rec and agl is not None and p is not None:
                hover.append((agl - h, drift(p0, p) if p0 is not None else np.nan))
            if not armed(s):
                return finish(rec, 'DISARMED', s, hmax, vmax, hover, stop, '飞控上锁了')
            if mode(s) not in ('', 'OFFBOARD'):
                return finish(rec, 'TAKEOVER', s, hmax, vmax, hover, stop, '模式变成 %s：飞手接管' % mode(s))
            if st == 'STOPPED' or s.get('gateway_status') == 'FAULT':
                return finish(rec, 'GATEWAY_FAULT', s, hmax, vmax, hover, stop, '网关停止起飞：%s。网关在当前位置刹停保持，请接管' % hint(
                    s.get('gateway_status', 'reason') or st))
            if now - last >= (0.5 if 'done_s' not in rec else 2.0):
                last = now
                print('%5.1f s｜%s｜%s｜离地 %s｜爬升 %s｜水平偏离 %s' % (
                    now - t0, st or '?', fc.landed_name(), '%.2f m' % agl if agl is not None else '?',
                    '%.2f m/s' % -v if v is not None else '?', '%.3f m' % drift(p0, p) if p is not None and p0 is not None else '?'), flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        finish(rec, 'INTERRUPTED', s, hmax, vmax, hover, stop, '停止监视（没动飞控模式，网关继续保持悬停）')
        raise


def finish(rec, end, s, hmax, vmax, hover, stop, msg):
    rec.update(max_agl=hmax, overshoot_m=hmax - rec['takeoff_height'], max_climb_mps=vmax)
    if hover:
        e = np.array(hover)
        rec.update(hover_s=len(e) * 0.05, hover_z_mean=float(np.nanmean(e[:, 0])), hover_z_absmax=float(np.nanmax(np.abs(e[:, 0]))),
                   hover_drift_max=float(np.nanmax(e[:, 1])))
    summary = '最高离地 %.2f m（超调 %+.2f），最大爬升 %.2f m/s%s%s' % (
        hmax, hmax - rec['takeoff_height'], vmax,
        '，离地用时 %.1f s' % rec['liftoff_s'] if 'liftoff_s' in rec else '，没看到离地',
        '；悬停 %.0f s 高度误差均值 %+.3f m、最大 %.3f m，水平偏离最大 %.3f m' % (
            rec['hover_s'], rec['hover_z_mean'], rec['hover_z_absmax'], rec['hover_drift_max']) if hover else '')
    return stop(end, '%s｜%s' % (msg, summary))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n停止', flush=True)
        sys.exit(130)
