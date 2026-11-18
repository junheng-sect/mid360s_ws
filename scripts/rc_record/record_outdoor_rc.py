#!/usr/bin/env python3
"""Drive the native rosbag recorder from the board's Channel 6 policy."""
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import String

TOPICS = (
    "/livox/lidar", "/livox/imu", "/Odometry", "/fastlio/gravity_init", "/fastlio/odom_world",
    "/tf", "/tf_static", "/drone_reloc/result", "/drone_reloc/status",
    "/mavros/state", "/mavros/local_position/odom", "/mavros/extended_state", "/mavros/timesync_status",
)
RECORDER = "/opt/ros/noetic/lib/rosbag/record"
DLL_TOPICS = ("/drone_reloc/pose", "/dll_localization/pose", "/dll_localization/correction",
              "/dll_localization/status")
YWL_TOPICS = ("/drone_reloc/pose", "/fastlio/velocity", "/ywl/bootstrap_odom",
              "/ywl/initial_transform", "/ywl/odom", "/ywl/velocity", "/ywl/status")
CHANNEL_INDEX = 5
THRESHOLD_US = 1650


class OutdoorRecorder:
    def __init__(self, directory, session, stopped, topics):
        self.directory = directory
        self.session = session
        self.stopped = stopped
        self.topics = topics
        self.lock = threading.Lock()
        self.process = self.path = self.error = None
        self.open_reported = False
        self.subscriber = rospy.Subscriber("/mavros/rc/in", RCIn, self.on_rc, queue_size=20,
                                           tcp_nodelay=True)

    def on_rc(self, message):
        with self.lock:
            if self.stopped.is_set():
                return
            valid = len(message.channels) > CHANNEL_INDEX
            active = valid and message.channels[CHANNEL_INDEX] >= THRESHOLD_US
            closing = self.session.close_due_at is not None
            action = self.session.on_rc(time.monotonic(), valid, active)
            if action == self.session.OPEN:
                try:
                    name = datetime.now().strftime("outdoor_dll_ywl_%Y%m%d_%H%M%S.bag")
                    path = self.directory / name
                    if path.exists() or path.with_suffix(".bag.active").exists():
                        raise FileExistsError(str(path))
                    self.process = subprocess.Popen([RECORDER, "-O", str(path), *self.topics],
                                                    start_new_session=True)
                    self.path = path
                    self.open_reported = False
                    self.session.bag_opened()
                    rospy.loginfo("Channel 6 触发录包：%s", path)
                except OSError as error:
                    self.error = str(error)
                    self.stopped.set()
            elif action == self.session.PENDING_CLOSE:
                rospy.loginfo("Channel 6 已离开录制位，约 1 秒后封包")
            elif closing and self.session.close_due_at is None and self.process is not None:
                rospy.loginfo("Channel 6 恢复录包")

    def close_bag(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try:
            code = self.process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.error = f"封包超时：{self.path}"
            try:
                code = self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                code = self.process.wait()
        if code == 0 and self.path.is_file():
            rospy.loginfo("已封包：%s", self.path)
        else:
            self.error = self.error or f"录制器退出码 {code}，请检查：{self.path}"
        self.process = self.path = None
        self.session.bag_closed()

    def run(self):
        try:
            while not self.stopped.wait(0.1) and not rospy.is_shutdown():
                with self.lock:
                    if self.process is not None and self.process.poll() is not None:
                        self.error = f"录制器意外退出：{self.path}"
                        self.stopped.set()
                    elif self.session.tick(time.monotonic()) == self.session.CLOSE:
                        self.close_bag()
                        if self.error:
                            self.stopped.set()
                    elif (self.path is not None and not self.open_reported
                          and self.path.with_suffix(".bag.active").is_file()):
                        self.open_reported = True
                        rospy.loginfo("Channel 6 录包中：%s", self.path)
        finally:
            self.stopped.set()
            with self.lock:
                self.close_bag()
            self.subscriber.unregister()
        if self.error:
            rospy.logerr("%s", self.error)
            return 1
        print("TEST12_RECORD stopped", flush=True)
        return 0


def main():
    directory, policy_dir, *options = sys.argv[1:]
    sys.path.insert(0, policy_dir)
    from rc_record_policy import RcRecordSession

    if not Path(RECORDER).is_file():
        raise SystemExit(f"找不到 ROS 录制器：{RECORDER}")
    rospy.init_node("outdoor_dll_ywl_record", anonymous=True, disable_signals=True)
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.set())
    rospy.loginfo("等待 Channel 6：先拨到关闭位，再拨到录制位。输出目录 %s", directory)
    session = RcRecordSession(confirm_samples=3, release_tail_s=1.0)
    optional = {(): (), ("--dll",): DLL_TOPICS, ("--ywl",): YWL_TOPICS}
    if tuple(options) not in optional:
        raise SystemExit("请选择 --dll 或 --ywl")
    topics = TOPICS + optional[tuple(options)]
    if tuple(options) != ("--dll",):
        return OutdoorRecorder(Path(directory), session, stopped, topics).run()

    publisher = rospy.Publisher("/test12/observer", String, queue_size=1, latch=True)
    note_path = Path(directory)/"dll_unavailable.txt"

    def announce(_event=None):
        try:
            note = note_path.read_text().strip()
        except OSError:
            note = ""
        publisher.publish(String(data=json.dumps(dict(mode="dll", run=directory, note=note), ensure_ascii=False)))

    announce()
    timer = rospy.Timer(rospy.Duration(1), announce)
    try:
        return OutdoorRecorder(Path(directory), session, stopped, topics + ("/test12/observer",)).run()
    finally:
        timer.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
