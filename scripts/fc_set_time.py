#!/usr/bin/env python3
"""飞控校时：趁 MAVROS 还没占用串口，把板端 UTC 时间写进飞控 RTC。

背景（2026-10-16 实测）：Pixhawk 6C 上 PX4 1.16 固件的 RTC 用 HSE 计时
（CONFIG_STM32H7_RTC_HSECLOCK），飞控断电时 RTC 停走，数值靠备份电源保留，开机后接着走，
所以每断一次电就慢一个断电时长。PX4 又只在自身时钟早于 2009 年时才接受 MAVROS
发来的 SYSTEM_TIME，板端时间进不去。这里改走 MAVLink shell（SERIAL_CONTROL / DEV_SHELL）
执行 NuttX 的 `date -u -s`，clock_settime 会同时写 RTC，软重启后仍保持。断电还是会少掉
断电时长，通电时也比板端慢约 33 ppm（2026-10-18 实测连续通电约 35.5 h 慢 4.29 s），
所以每次启动链路前写一次。

只影响 ulog 文件名、日期这类 UTC 时间；EKF 融合用的是飞控开机时间 + TIMESYNC，不受影响。

用法: fc_set_time.py [--dev /dev/ttyS3] [--baud 921600] [--tolerance 1.0] [--check]
  --tolerance  偏差在这个范围内（秒）就不写；date -s 只有秒精度，默认 1.0
  --check      只读偏差，不写
退出码: 0 已写入并核对通过，或本来就在容差内 / --check 读到了偏差
        1 失败（没心跳、没 SYSTEM_TIME、shell 无回显、核对不过、板端时间明显不对）
        2 飞控已解锁，跳过    3 缺 pymavlink    143 被 SIGTERM 停下（已关掉 shell）
直接占用串口，MAVROS 运行时不要用。板端时间是否可信由调用方负责：fusion.sh 只在
chrony 已对上时才调用。
"""

import argparse
import datetime as dt
import math
import signal
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("缺 pymavlink（python3 -m pip install --user pymavlink）")
    sys.exit(3)

M = mavutil.mavlink
SHELL_FLAGS = (M.SERIAL_CONTROL_FLAG_RESPOND | M.SERIAL_CONTROL_FLAG_EXCLUSIVE |
               M.SERIAL_CONTROL_FLAG_MULTI)
BOARD_TIME_SANE = 1.7e9   # 2023-11 以前的板端时间肯定不对，不往飞控写


def fail(msg):
    print(msg)
    sys.exit(1)


def wait_autopilot_heartbeat(m, timeout):
    t_end = time.time() + timeout
    while time.time() < t_end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=max(0.1, t_end - time.time()))
        if hb is not None and hb.autopilot != M.MAV_AUTOPILOT_INVALID:
            return hb
    return None


def clock_offset(m, sysid, n=3, timeout=5.0):
    """收 n 个飞控 SYSTEM_TIME，返回 飞控 - 板端 偏差（秒）的中位数；收不到返回 None"""
    offs, t_end = [], time.time() + timeout
    while len(offs) < n and time.time() < t_end:
        msg = m.recv_match(type="SYSTEM_TIME", blocking=True, timeout=max(0.1, t_end - time.time()))
        if msg is not None and msg.get_srcSystem() == sysid:
            offs.append(msg.time_unix_usec / 1e6 - time.time())
    return sorted(offs)[len(offs) // 2] if offs else None


def shell_send(m, line, flags=SHELL_FLAGS):
    # pymavlink 默认按 MAVLink v1 生成，没有 target_system/component 扩展字段；
    # PX4 把缺省的 0 当作广播，照样处理
    data = line.encode()
    for i in range(0, max(len(data), 1), 70):
        chunk = list(data[i:i + 70])
        m.mav.serial_control_send(M.SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, len(chunk),
                                  chunk + [0] * (70 - len(chunk)))


def shell_read(m, duration):
    out, t_end = b"", time.time() + duration
    while time.time() < t_end:
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.1)
        if msg is not None and msg.count:
            out += bytes(msg.data[:msg.count])
    return out.decode(errors="replace")


def fmt_off(off):
    return "%+.3f s" % off if abs(off) < 3600 else "%+.3f s（%+.2f h）" % (off, off / 3600)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--dev", default="/dev/ttyS3")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--tolerance", type=float, default=1.0)
    p.add_argument("--check", action="store_true")
    a = p.parse_args()

    if time.time() < BOARD_TIME_SANE:
        fail("板端时间明显不对（%s），不往飞控写" % time.strftime("%F %T"))
    try:
        m = mavutil.mavlink_connection(a.dev, baud=a.baud, source_system=255,
                                       source_component=M.MAV_COMP_ID_MISSIONPLANNER,
                                       autoreconnect=False)
    except Exception as e:  # 串口打不开（权限、被占用）
        fail("打不开 %s：%s" % (a.dev, e))

    hb = wait_autopilot_heartbeat(m, 5.0)
    if hb is None:
        fail("5 s 内没收到飞控心跳（%s @ %d）" % (a.dev, a.baud))
    sysid = hb.get_srcSystem()
    before = clock_offset(m, sysid)
    if before is None:
        fail("5 s 内没收到飞控 SYSTEM_TIME")
    print("飞控 - 板端 = %s" % fmt_off(before))
    if a.check:
        return 0
    if abs(before) <= a.tolerance:
        print("已在 %.1f s 容差内，不写" % a.tolerance)
        return 0
    if hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
        print("飞控已解锁，不改时间")
        return 2

    # 唤醒 shell，确认有回显再写。PX4 只在收到不带 RESPOND 的 SERIAL_CONTROL 时才关 shell，
    # 所以中途失败或被 SIGTERM/Ctrl+C 打断也要发这一条，不然飞控上的 nsh 会一直开着
    shell_send(m, "\n")
    try:
        if "nsh>" not in shell_read(m, 1.0):
            fail("MAVLink shell 没有回显（没等到 nsh> 提示符）")
        # 卡在整秒边界发出：nsh 收到换行即执行，date -s 只有秒精度
        target = math.ceil(time.time()) + 1
        cmd = 'date -u -s "%s"' % dt.datetime.fromtimestamp(target, dt.timezone.utc).strftime("%b %d %H:%M:%S %Y")
        while time.time() < target:
            time.sleep(0.0005)
        shell_send(m, cmd + "\n")
        echo = shell_read(m, 0.5)
    finally:
        shell_send(m, "", flags=0)
    if "failed" in echo or "Invalid" in echo:
        fail("date 执行报错：%r" % echo)

    after = clock_offset(m, sysid)
    m.close()
    if after is None:
        fail("写入后没收到 SYSTEM_TIME，无法核对")
    if abs(after) > a.tolerance:
        fail("写入后偏差仍有 %s（写入前 %s）" % (fmt_off(after), fmt_off(before)))
    print("已写入：%s → %s（%s）" % (fmt_off(before), fmt_off(after), cmd))
    return 0


if __name__ == "__main__":
    # fusion.sh 收尾时用 SIGTERM 停本脚本：转成 SystemExit，好让上面的 finally 关掉 shell
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    sys.exit(main())
