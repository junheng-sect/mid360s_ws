#!/usr/bin/env python3
"""定位核心数据连续性监控：只读消息头、不解析点云，进程活着不代表定位有效，这里盯数据本身。

fusion.sh 在 roscore 起来后自动拉起，事件和汇总进 fusion 日志，逐秒明细写 CSV；也可以单独跑：
  source ~/mid360s_ws/scripts/ros_net.sh && source /opt/ros/noetic/setup.bash
  python3 ~/mid360s_ws/scripts/core_watch.py [--csv 文件] [--summary-sec 60] [--ev-mode pose|odom]

- 每个话题：频率、相邻两帧接收最大间隔、消息时间戳是否倒退、时间戳落后接收时刻多少（lag）。
  话题流起来以后断流超过阈值报 STALE，恢复报 RESUMED；从没来过的只在汇总里列出（雷达没上电、
  初始化没过时本来就没有）。
- 系统时间相对单调时钟的跳变：核心运行中 fusion.sh 已冻结时钟，出现跳变就是问题。
- 飞控侧：/mavros/local_position/pose 还在不在更新，以及它和送给飞控的视觉位姿之间的距离 ev_resid。
  送给飞控的视觉位姿按 --ev-mode 取：pose 方式是 /mavros/vision_pose/pose_cov，odom 方式是 /mavros/odometry/out。
  外部定位在融合时两者应在厘米级一致，持续变大说明飞控没在用（或用不上）视觉位姿。
  两者原点都是起飞前初始化处、都是 ENU，未融合前（刚上电）可能有固定偏差，看变化趋势。
"""
import argparse
import csv
import math
import struct
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# (话题, 断流阈值 s)；消息第一个字段都是 std_msgs/Header，AnyMsg 直接从原始字节读 seq / stamp
TOPICS = [
    ('/livox/lidar', 0.3),
    ('/livox/imu', 0.1),
    ('/Odometry', 0.3),
    ('/fastlio/odom_world', 0.3),
    ('/mavros/vision_pose/pose_cov', 0.3),
    ('/mavros/local_position/pose', 0.3),
    ('/mavros/state', 3.0),
]
LOCAL = '/mavros/local_position/pose'
# 送给飞控的视觉位姿：TOPICS 里写的是 pose 方式的话题，odom 方式换成 /mavros/odometry/out
VISION_TOPICS = {'pose': ('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped),
                 'odom': ('/mavros/odometry/out', Odometry)}
HEADER = struct.Struct('<III')   # seq, stamp.secs, stamp.nsecs
CLOCK_JUMP_S = 0.05              # 系统时间相对单调时钟一次变化超过它算跳变
LAG_WARN_S = 1.0                 # 时间戳落后接收时刻超过它告警（时间不一致或严重积压）


def say(msg):
    print('[core_watch %s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)


class Topic:
    def __init__(self, name, stale_s):
        self.name, self.stale_s = name, stale_s
        self.n = self.win_n = 0
        self.first_rx = self.last_rx = None      # time.monotonic()
        self.last_stamp = None
        self.win_gap = self.max_gap = 0.0
        self.win_lag = self.max_lag = 0.0
        self.backwards = self.episodes = 0
        self.stale = self.lag_warned = False

    def rx(self, stamp, mono, wall):
        if self.last_rx is not None:
            gap = mono - self.last_rx
            self.win_gap, self.max_gap = max(self.win_gap, gap), max(self.max_gap, gap)
            if self.stale:
                self.stale = False
                say('RESUMED %s（断流 %.2f s）' % (self.name, gap))
        else:
            self.first_rx = mono
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.backwards += 1
            say('STAMP BACKWARDS %s：%.6f < %.6f（第 %d 次）' % (self.name, stamp, self.last_stamp, self.backwards))
        if stamp > 0:
            lag = wall - stamp
            self.win_lag, self.max_lag = max(self.win_lag, lag), max(self.max_lag, lag)
            if abs(lag) > LAG_WARN_S and not self.lag_warned:
                self.lag_warned = True
                say('LAG %s：时间戳与接收时刻差 %.3f s（时间不一致或积压）' % (self.name, lag))
        self.last_rx, self.last_stamp = mono, stamp
        self.n += 1
        self.win_n += 1

    def check_stale(self, mono):
        if self.last_rx is not None and not self.stale and mono - self.last_rx > self.stale_s:
            self.stale = True
            self.episodes += 1
            say('STALE %s：%.2f s 没有新消息（阈值 %.2f s）' % (self.name, mono - self.last_rx, self.stale_s))

    def take_window(self, mono):
        row = (self.win_n, self.win_gap, (mono - self.last_rx) if self.last_rx is not None else '',
               self.win_lag if self.win_n else '')
        self.win_n, self.win_gap, self.win_lag = 0, 0.0, 0.0
        return row


class Watch:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.vision, vision_type = VISION_TOPICS[args.ev_mode]
        pose_topic = VISION_TOPICS['pose'][0]
        self.topic_list = [(self.vision if name == pose_topic else name, s) for name, s in TOPICS]
        self.topics = {name: Topic(name, s) for name, s in self.topic_list}
        self.pos = {}                            # 话题 -> (单调时刻, x, y, z)
        self.win_resid = self.max_resid = None
        self.resid_warned = False
        self.clock_jumps = 0
        self.max_clock_step = 0.0
        self.t0 = time.monotonic()
        for name, _ in self.topic_list:
            if name == self.vision:
                # PoseWithCovarianceStamped 和 Odometry 的位置都在 msg.pose.pose.position
                rospy.Subscriber(name, vision_type, self.on_pose, (name, True), queue_size=50)
            elif name == LOCAL:
                rospy.Subscriber(name, PoseStamped, self.on_pose, (name, False), queue_size=50)
            else:
                rospy.Subscriber(name, rospy.AnyMsg, self.on_any, name, queue_size=50)
        self.csv = None
        if args.csv:
            self.csv_file = open(args.csv, 'w', newline='', buffering=1)
            self.csv = csv.writer(self.csv_file)
            head = ['wall', 'mono_s']    # mono_s：开机以来的单调时间，和 /proc/uptime、journal 对得上
            for name, _ in self.topic_list:
                k = name.strip('/').replace('/', '.')
                head += [k + '.n', k + '.max_gap', k + '.age', k + '.max_lag']
            self.csv.writerow(head + ['ev_resid_max', 'clock_step'])

    def on_any(self, msg, name):
        mono, wall = time.monotonic(), time.time()
        try:
            _, secs, nsecs = HEADER.unpack_from(msg._buff, 0)
        except struct.error:
            return
        with self.lock:
            self.topics[name].rx(secs + nsecs * 1e-9, mono, wall)

    def on_pose(self, msg, cb_args):
        name, with_cov = cb_args
        mono, wall = time.monotonic(), time.time()
        p = msg.pose.pose.position if with_cov else msg.pose.position
        with self.lock:
            self.topics[name].rx(msg.header.stamp.to_sec(), mono, wall)
            self.pos[name] = (mono, p.x, p.y, p.z)
            self.update_resid(mono)

    def update_resid(self, mono):
        a, b = self.pos.get(self.vision), self.pos.get(LOCAL)
        if a is None or b is None or abs(a[0] - b[0]) > 0.5 or mono - min(a[0], b[0]) > 0.5:
            return
        d = math.sqrt(sum((a[i] - b[i]) ** 2 for i in (1, 2, 3)))
        self.win_resid = d if self.win_resid is None else max(self.win_resid, d)
        self.max_resid = d if self.max_resid is None else max(self.max_resid, d)
        if d > self.args.resid_warn and not self.resid_warned:
            self.resid_warned = True
            say('EV_RESID 飞控本地位置与视觉位姿相差 %.2f m（阈值 %.2f m）：飞控可能没在融合视觉位姿'
                % (d, self.args.resid_warn))
        elif d < self.args.resid_warn * 0.5 and self.resid_warned:
            self.resid_warned = False
            say('EV_RESID 恢复：相差 %.2f m' % d)

    def summary(self, final=False):
        up = time.monotonic() - self.t0
        say('%s（运行 %.0f s，系统时间跳变 %d 次，最大单次 %.3f s，ev_resid 最大 %s）' % (
            '最终汇总' if final else '汇总', up, self.clock_jumps, self.max_clock_step,
            '%.3f m' % self.max_resid if self.max_resid is not None else '无'))
        for name, _ in self.topic_list:
            t = self.topics[name]
            if t.first_rx is None:
                say('  %-30s 从没收到' % name)
                continue
            span = max(t.last_rx - t.first_rx, 1e-6)
            say('  %-30s %7.1f Hz  最大间隔 %.3f s  断流 %d 次  时间戳倒退 %d 次  最大 lag %.3f s%s' % (
                name, (t.n - 1) / span, t.max_gap, t.episodes, t.backwards, t.max_lag,
                '  [当前断流]' if t.stale else ''))

    def run(self):
        wall_prev, mono_prev = time.time(), time.monotonic()
        next_row, next_sum = mono_prev + 1.0, mono_prev + self.args.summary_sec
        clock_step = 0.0
        while not rospy.is_shutdown():
            time.sleep(0.05)
            wall, mono = time.time(), time.monotonic()
            step = (wall - wall_prev) - (mono - mono_prev)
            wall_prev, mono_prev = wall, mono
            if abs(step) > CLOCK_JUMP_S:
                with self.lock:
                    self.clock_jumps += 1
                    self.max_clock_step = max(self.max_clock_step, abs(step))
                clock_step += step
                say('CLOCK JUMP 系统时间相对单调时钟跳了 %+.3f s' % step)
            with self.lock:
                for t in self.topics.values():
                    t.check_stale(mono)
                if mono >= next_row:
                    next_row += 1.0
                    if self.csv:
                        row = [time.strftime('%H:%M:%S'), '%.1f' % mono]
                        for name, _ in self.topic_list:
                            row += ['%.3f' % v if isinstance(v, float) else v
                                    for v in self.topics[name].take_window(mono)]
                        row += ['%.3f' % self.win_resid if self.win_resid is not None else '', '%.3f' % clock_step]
                        self.csv.writerow(row)
                    else:
                        for t in self.topics.values():
                            t.take_window(mono)
                    self.win_resid, clock_step = None, 0.0
                if mono >= next_sum:
                    next_sum += self.args.summary_sec
                    self.summary()
        with self.lock:
            self.summary(final=True)
        if self.csv:
            self.csv_file.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--csv', help='逐秒明细 CSV 路径（不给就不写）')
    ap.add_argument('--summary-sec', type=float, default=60.0, help='汇总间隔（秒），默认 60')
    ap.add_argument('--resid-warn', type=float, default=0.5,
                    help='飞控本地位置与视觉位姿相差超过它告警（米），默认 0.5')
    ap.add_argument('--ev-mode', choices=sorted(VISION_TOPICS), default='pose',
                    help='外部定位送飞控的方式（与 fusion.sh --ev-mode 一致），决定盯哪个视觉位姿话题')
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('core_watch')
    w = Watch(args)
    say('开始监控：%s；CSV %s' % (' '.join(n for n, _ in w.topic_list), args.csv or '不写'))
    w.run()


if __name__ == '__main__':
    main()
