#!/usr/bin/env python3
"""Follow CH6 bag events; only the C++ child subscribes to raw data.

The existing RC policy owns debounce, the release tail and RC-loss handling.
NAME_rc_NNNNNN.bag is paired with NAME_raw_NNNNNN.bag. Child shutdown is
asynchronous so another CH6 segment can start while the previous bag closes.
This controller stays outside the required flight roslaunch.
"""
import argparse
import json
import os
from pathlib import Path
import queue
import re
import signal
import socket
import subprocess
import threading
import time
from urllib.parse import urlparse

RAW_TOPICS = ('/livox/lidar', '/livox/imu', '/tf_static',
              '/fastlio/gravity_init', '/Odometry')


class RawBagRecorder:
    def __init__(self, run_dir, bag_dir, prefix, record_bin):
        self.run_dir = Path(run_dir)
        self.bag_dir = Path(bag_dir).resolve()
        self.prefix = prefix
        self.record_bin = record_bin
        self.events = queue.Queue()
        self.stop = threading.Event()
        self.active = None
        self.sessions = []
        self.seen = set()
        self.errors = []
        self.last_status = None
        self.master_lost = None
        self.finished = False
        self.pattern = re.compile(re.escape(prefix) + r'_rc_(\d{6,})\.bag\Z')

    @staticmethod
    def log(message):
        print('[%s] [raw_record] %s' % (time.strftime('%H:%M:%S'), message), flush=True)

    def error(self, message):
        self.errors.append(message)
        self.log('[ERROR] ' + message)

    def raw_path(self, source):
        path = Path(source)
        match = self.pattern.fullmatch(path.name)
        if not match or path.parent.resolve() != self.bag_dir:
            return None
        return self.bag_dir / ('%s_raw_%s.bag' % (self.prefix, match.group(1)))

    def request_close(self, session):
        if session['stop_at'] is None:
            session['stop_at'] = time.monotonic()
            if session['process'].poll() is None:
                session['process'].send_signal(signal.SIGINT)
            if self.active is session:
                self.active = None

    def open(self, source):
        raw = self.raw_path(source)
        if raw is None or source in self.seen:
            return
        self.seen.add(source)  # Delayed REC status must not reopen a closed bag.
        if self.active is not None:
            self.request_close(self.active)
        if raw.exists() or Path(str(raw) + '.active').exists():
            self.error('拒绝覆盖已有原始包：%s' % raw)
            return
        try:
            child = subprocess.Popen(
                [self.record_bin, '-O', str(raw)] + list(RAW_TOPICS),
                stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as exc:
            self.error('无法启动 C++ record：%s' % exc)
            return
        session = {'source': source, 'path': str(raw), 'process': child,
                   'stop_at': None, 'term_sent': False, 'kill_sent': False,
                   'done': False, 'status_count': None}
        self.sessions.append(session)
        self.active = session
        with (self.run_dir / 'raw.paths').open('a') as stream:
            stream.write(str(raw) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        self.log('CH6 开：启动 C++ 原始点云录包 %s；起飞前保持静止 5–10 s' % raw)

    def event(self, action, source):
        if self.raw_path(source) is None:
            return
        if action == 'OPEN':
            self.open(source)
        elif action in ('CLOSED', 'CLOSE_FAILED'):
            if self.active is not None and self.active['source'] == source:
                self.request_close(self.active)
            self.seen.add(source)
            if action == 'CLOSE_FAILED':
                self.error('轻量包关闭失败：%s' % source)

    def poll(self):
        now = time.monotonic()
        for session in self.sessions:
            if session['done']:
                continue
            child = session['process']
            code = child.poll()
            if code is not None:
                session['done'] = True
                if self.active is session:
                    self.active = None
                if session['stop_at'] is None:
                    self.error('C++ record 提前退出（%s）：%s' % (code, session['path']))
                elif code != 0 or not Path(session['path']).is_file():
                    self.error('原始包未正常收尾（%s）：%s' % (code, session['path']))
                else:
                    self.log('CH6 关：原始包已收尾 %s' % session['path'])
                continue
            if session['stop_at'] is None:
                continue
            elapsed = now - session['stop_at']
            if elapsed >= 25 and not session['kill_sent']:
                session['kill_sent'] = True
                self.error('record 收尾超时，强制停止；保留 .active 供恢复：%s' % session['path'])
                child.kill()
            elif elapsed >= 20 and not session['term_sent']:
                session['term_sent'] = True
                self.error('record 20 s 未退出，发送 SIGTERM：%s' % session['path'])
                child.terminate()

    def status(self, text):
        # Existing rc_record status format; paths may contain spaces.
        match = re.fullmatch(r'state=(REC|IDLE) bags=(\d+) last=(.*?) '
                             r'rc_valid=(True|False) healthy=(True|False)', text)
        if not match:
            return
        self.last_status = time.monotonic()
        state, count, source, _, healthy = match.groups()
        if healthy == 'False':
            if self.active is not None:
                self.request_close(self.active)
            self.error('CH6 轻量录包节点报告故障；停止原始录包，不停止定位链路')
            self.stop.set()
        elif state == 'REC':
            self.event('OPEN', source)  # Recovers a missed initial OPEN event.
            if self.active is not None and self.active['source'] == source:
                self.active['status_count'] = int(count)
        elif (self.active is not None and self.active['status_count'] is not None
              and int(count) >= self.active['status_count']):
            # An IDLE from before OPEN can arrive later on the other topic.
            self.request_close(self.active)

    def drain(self):
        for _ in range(100):
            try:
                kind, text = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'path':
                action, _, source = text.partition(' ')
                self.event(action, source)
            else:
                self.status(text)

    def finish(self):
        if self.finished:
            return
        self.finished = True
        for session in self.sessions:
            if not session['done']:
                self.request_close(session)
        while any(not item['done'] for item in self.sessions):
            self.poll()
            time.sleep(.05)
        try:
            subprocess.run(['sync', '-f', str(self.bag_dir)], check=True, timeout=15)
        except (OSError, subprocess.SubprocessError) as exc:
            self.error('原始包同步落盘失败：%s' % exc)
        summary = {'paths': [item['path'] for item in self.sessions], 'errors': self.errors}
        temporary = self.run_dir / 'raw.summary.tmp'
        temporary.write_text(json.dumps(summary, ensure_ascii=False) + '\n')
        temporary.replace(self.run_dir / 'raw.summary.json')
        self.log('原始录包控制已退出，%d 段，%d 个错误' %
                 (len(self.sessions), len(self.errors)))


def master_online():
    uri = urlparse(os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311'))
    try:
        with socket.create_connection((uri.hostname, uri.port or 11311), timeout=.3):
            return True
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--bag-dir', required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--record-bin', default='/opt/ros/noetic/lib/rosbag/record')
    args = parser.parse_args()
    recorder = RawBagRecorder(args.run_dir, args.bag_dir, args.prefix, args.record_bin)
    start = Path('/proc/%d/stat' % os.getpid()).read_text().rsplit(')', 1)[1].split()[19]
    (recorder.run_dir / 'raw.pid').write_text('%d %s\n' % (os.getpid(), start))
    signal.signal(signal.SIGINT, lambda *_: recorder.stop.set())
    signal.signal(signal.SIGTERM, lambda *_: recorder.stop.set())
    signal.signal(signal.SIGHUP, lambda *_: recorder.stop.set())
    try:
        while not recorder.stop.is_set() and not master_online():
            recorder.stop.wait(.2)
        if recorder.stop.is_set():
            return 0
        import rospy
        from std_msgs.msg import String
        rospy.init_node('map_raw_record_control', anonymous=True, disable_signals=True)
        path_sub = rospy.Subscriber('/rc_record/bag_path', String,
                                    lambda msg: recorder.events.put(('path', msg.data)),
                                    queue_size=100, tcp_nodelay=True)
        status_sub = rospy.Subscriber('/rc_record/status', String,
                                      lambda msg: recorder.events.put(('status', msg.data)),
                                      queue_size=100, tcp_nodelay=True)
        (recorder.run_dir / 'raw.ready').touch()
        recorder.log('CH6 原始录包待命；先将 CH6 置于关闭位，再拨开启位开始所有录包')
        last_check = time.monotonic()
        while not recorder.stop.is_set() and not rospy.is_shutdown():
            recorder.drain()
            recorder.poll()
            now = time.monotonic()
            if now - last_check >= 2:
                last_check = now
                if master_online():
                    recorder.master_lost = None
                elif recorder.master_lost is None:
                    recorder.master_lost = now
                elif now - recorder.master_lost >= 10:
                    recorder.log('ROS master 持续离线，收尾原始包')
                    break
                if recorder.last_status is not None and now - recorder.last_status >= 10:
                    recorder.error('CH6 录包状态超过 10 s 没有更新，收尾原始包')
                    break
            recorder.stop.wait(.1)
        path_sub.unregister()
        status_sub.unregister()
    except Exception as exc:
        recorder.error('原始录包控制异常：%s' % exc)
    finally:
        recorder.finish()
    return 1 if recorder.errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
