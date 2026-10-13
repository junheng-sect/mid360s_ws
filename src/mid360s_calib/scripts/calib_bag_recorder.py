#!/usr/bin/env python3
"""标定专用 RC 开关触发录包节点（iKalibr 外参标定数据）。

只录标定需要的 4 个话题：
  /livox/lidar             livox_ros_driver2/CustomMsg   原始点云（iKalibr LIVOX_CUSTOM）
  /livox/imu               sensor_msgs/Imu               雷达内置 IMU，加速度单位 g（SENSOR_IMU_G）
  /mavros/imu/data_raw     sensor_msgs/Imu               飞控 IMU，m/s²、FLU（SENSOR_IMU，作 ReferIMU）
  /mavros/timesync_status  mavros_msgs/TimesyncStatus    飞控时间同步质量，事后核对时间戳用

触发与平时飞行的 rc_record 相同：/mavros/rc/in 通道 6，PWM >= 1650 为触发位，去抖 + 1 s 尾巴；
开关策略 RcRecordSession 与序号分配 SequenceAllocator 原样复制自 scripts/rc_record。

飞控 IMU 只在录包期间提频：开包后调用 /mavros/set_message_interval 请求 HIGHRES_IMU(105)
@ fcu_imu_rate_hz（默认 400，板端实测约 190 Hz；0 表示不改），关包后恢复默认（message_rate=0）；
录包期间 MAVROS 重连（飞控重启会丢掉该设置）则重新请求。服务调用都在单独的工作线程里，
不阻塞写包。空闲和退出时飞控已经是默认频率，不依赖退出那一刻 MAVROS 还活着
（roslaunch 并行停节点，退出时再调服务常常赶上 MAVROS 已先退出）。

关包时打印体检摘要（每个话题条数与平均频率）；话题缺失、飞控 IMU 低于 min_fcu_imu_hz 时告警。

话题：
  订阅  <rc_topic> mavros_msgs/RCIn、/mavros/state、上述 4 个数据话题
  发布  <bag_path_topic> (latched) std_msgs/String  "OPEN <path>" / "CLOSED <path>"
        <status_topic>             std_msgs/String  状态
  调用  /mavros/set_message_interval  mavros_msgs/MessageInterval
"""

import os
import signal
import sys
import threading
import time

import rosbag
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from mavros_msgs.msg import RCIn, State, TimesyncStatus
from mavros_msgs.srv import MessageInterval

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rc_record_policy import RcRecordSession  # noqa: E402
from sequence_allocator import SequenceAllocator  # noqa: E402

TICK_PERIOD = 0.2            # 5 Hz 心跳：保证尾巴到期有人关包
RATE_WORKER_PERIOD = 0.2     # 提频/恢复请求的检查周期
FCU_IMU_TOPIC = '/mavros/imu/data_raw'
SET_INTERVAL_SERVICE = '/mavros/set_message_interval'
HIGHRES_IMU_ID = 105


def calib_topics():
    """标定包的话题与消息类型（CustomMsg 需要 source mid360s_ws/devel/setup.bash）。"""
    from livox_ros_driver2.msg import CustomMsg
    return [
        ('/livox/lidar', CustomMsg),
        ('/livox/imu', Imu),
        (FCU_IMU_TOPIC, Imu),
        ('/mavros/timesync_status', TimesyncStatus),
    ]


def message_stamp(message):
    """bag 时间戳：有 header.stamp 且非零就用它，否则用接收时刻（与 rosbag record 同口径）。"""
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None) if header is not None else None
    if stamp is not None and (stamp.secs or stamp.nsecs):
        return stamp
    return rospy.Time.now()


def fcu_rate_request(want_boost, applied, connected, reapply, boost_hz):
    """决定这一拍要不要调 set_message_interval。

    返回要请求的频率（boost_hz 提频，0.0 恢复默认），不需要调用时返回 None。
    飞控没连上时不调（调了也只会超时），等重连后再补。
    """
    if boost_hz <= 0.0 or not connected:
        return None
    if want_boost and (not applied or reapply):
        return boost_hz
    if not want_boost and applied:
        return 0.0
    return None


def bag_health(counts, duration_s, min_fcu_imu_hz):
    """每个话题的平均频率，以及需要告警的问题列表。

    counts: {topic: 条数}；min_fcu_imu_hz <= 0 时不检查飞控 IMU 频率。
    """
    rates = {topic: (count / duration_s if duration_s > 0.0 else 0.0)
             for topic, count in counts.items()}
    problems = ['%s 没有收到任何消息' % topic
                for topic, count in sorted(counts.items()) if count == 0]
    fcu_rate = rates.get(FCU_IMU_TOPIC, 0.0)
    if min_fcu_imu_hz > 0.0 and counts.get(FCU_IMU_TOPIC, 0) > 0 and fcu_rate < min_fcu_imu_hz:
        problems.append('%s 平均 %.1f Hz，低于 %.0f Hz' % (FCU_IMU_TOPIC, fcu_rate, min_fcu_imu_hz))
    return rates, problems


class CalibBagRecorder(object):
    def __init__(self):
        self._lock = threading.RLock()
        self._bag = None
        self._bag_path = ''
        self._bag_count = 0
        self._bag_opened_at = None
        self._counts = {}
        self._started_at = time.monotonic()
        self._last_rc_time = None
        self._rc_warned = False
        self._last_logged_pwm = None
        self._last_log_time = 0.0
        self._tick_count = 0

        # 飞控 IMU 提频状态（_lock 保护）
        self._connected = False
        self._want_boost = False
        self._boost_applied = False
        self._reapply = False
        self._rate_warned = False

        self._rc_topic = rospy.get_param('~rc_topic', '/mavros/rc/in')
        self._rc_channel = int(rospy.get_param('~rc_channel', 6))
        self._rc_threshold = int(rospy.get_param('~rc_threshold', 1650))
        self._active_high = bool(rospy.get_param('~active_high', True))
        self._rc_warn_timeout = float(rospy.get_param('~rc_warn_timeout_s', 5.0))
        self._stop_on_rc_loss = bool(rospy.get_param('~stop_on_rc_loss', False))
        self._bag_directory = os.path.abspath(
            rospy.get_param('~bag_directory', '/home/orangepi/test_data/bags/calib'))
        self._bag_prefix = rospy.get_param('~bag_prefix', 'calib')
        self._status_rate = float(rospy.get_param('~status_rate', 2.0))
        self._boost_hz = float(rospy.get_param('~fcu_imu_rate_hz', 400.0))
        self._min_fcu_imu_hz = float(rospy.get_param('~min_fcu_imu_hz', 150.0))
        if self._boost_hz <= 0.0:
            self._min_fcu_imu_hz = 0.0   # 不提频就不按高频标准体检

        self._session = RcRecordSession(
            confirm_samples=int(rospy.get_param('~confirm_samples', 3)),
            release_tail_s=float(rospy.get_param('~release_tail_s', 1.0)))

        try:
            self._topics = calib_topics()
        except ImportError as exception:
            rospy.logfatal('导入不到 livox_ros_driver2/CustomMsg（需要 source '
                           'mid360s_ws/devel/setup.bash）：%s', exception)
            rospy.signal_shutdown('livox CustomMsg unavailable')
            raise

        self._path_publisher = rospy.Publisher(
            rospy.get_param('~bag_path_topic', '/calib_record/bag_path'),
            String, queue_size=1, latch=True)
        self._status_publisher = rospy.Publisher(
            rospy.get_param('~status_topic', '/calib_record/status'),
            String, queue_size=1)
        self._set_interval = rospy.ServiceProxy(SET_INTERVAL_SERVICE, MessageInterval)

        self._subscribers = [
            rospy.Subscriber(topic, message_type, self._data_callback,
                             callback_args=topic, queue_size=1000, tcp_nodelay=True)
            for topic, message_type in self._topics
        ]
        self._rc_subscriber = rospy.Subscriber(
            self._rc_topic, RCIn, self._rc_callback, queue_size=20, tcp_nodelay=True)
        self._state_subscriber = rospy.Subscriber(
            '/mavros/state', State, self._state_callback, queue_size=5)
        self._timer = rospy.Timer(rospy.Duration(TICK_PERIOD), self._tick)

        self._stop = threading.Event()
        self._rate_thread = threading.Thread(target=self._rate_worker, name='fcu_imu_rate')
        self._rate_thread.daemon = True
        self._rate_thread.start()

        rospy.loginfo('标定录包就绪：%s 的通道 %d，阈值 %d us，active_high=%s，确认 %d 拍，'
                      '尾巴 %.1f s，输出 %s/%s_*.bag',
                      self._rc_topic, self._rc_channel, self._rc_threshold, self._active_high,
                      self._session.confirm_samples, self._session.release_tail_s,
                      self._bag_directory, self._bag_prefix)
        rospy.loginfo('录制话题（%d 个）：%s', len(self._topics),
                      ', '.join(t for t, _ in self._topics))
        if self._boost_hz > 0.0:
            rospy.loginfo('飞控 IMU：录包期间请求消息 %d @ %.0f Hz，关包后恢复默认',
                          HIGHRES_IMU_ID, self._boost_hz)
        else:
            rospy.logwarn('fcu_imu_rate_hz=0：不改飞控 IMU 频率（默认 50 Hz，标定精度会受影响）')
        self._publish_status()

    # ------------------------------------------------------------ RC ----
    def _rc_callback(self, message):
        index = self._rc_channel - 1
        now = time.monotonic()
        if index < 0 or index >= len(message.channels):
            with self._lock:
                self._rc_lost(now, 'RC 报文里没有通道 %d' % self._rc_channel)
            return
        pwm = int(message.channels[index])
        active = (pwm >= self._rc_threshold) if self._active_high \
            else (pwm <= self._rc_threshold)
        with self._lock:
            self._last_rc_time = now
            self._rc_warned = False
            if self._last_logged_pwm is None or abs(pwm - self._last_logged_pwm) >= 25:
                if now - self._last_log_time >= 1.0:
                    rospy.loginfo('通道 %d PWM=%d us -> 触发=%s（阈值 %d，%s）',
                                  self._rc_channel, pwm, active, self._rc_threshold,
                                  'high' if self._active_high else 'low')
                    self._last_logged_pwm = pwm
                    self._last_log_time = now
            self._apply(self._session.on_rc(now, True, active))

    def _rc_lost(self, now, reason):
        if not self._rc_warned:
            rospy.logwarn('收不到 RC 数据（%s，>%.1fs）；正在录制的包继续录，不会截断',
                          reason, self._rc_warn_timeout)
            self._rc_warned = True
        if self._stop_on_rc_loss:
            self._apply(self._session.note_rc_loss())

    def _tick(self, _event):
        now = time.monotonic()
        with self._lock:
            if self._last_rc_time is not None and \
                    now - self._last_rc_time > self._rc_warn_timeout:
                self._rc_lost(now, '%s 已静默' % self._rc_topic)
            elif self._last_rc_time is None and not self._rc_warned and \
                    now - self._started_at > self._rc_warn_timeout:
                # 从启动起就没收到过 RC（遥控器没开 / 没绑定）：拨开关也不会录包，提醒一次
                rospy.logwarn('启动 %.0f s 仍未收到 %s：遥控器没开或没绑定？此时拨开关不会录包',
                              self._rc_warn_timeout, self._rc_topic)
                self._rc_warned = True
            self._apply(self._session.tick(now))
            self._tick_count += 1
            if self._tick_count * TICK_PERIOD * self._status_rate >= 1.0:
                self._tick_count = 0
                self._publish_status()

    # ------------------------------------------------------ 状态机动作 ----
    def _apply(self, action):
        if action == RcRecordSession.OPEN:
            self._open_bag()
        elif action == RcRecordSession.PENDING_CLOSE:
            rospy.logwarn('触发位已释放：%.1f s 后关包（期间再次拨下会继续录同一个包）',
                          self._session.release_tail_s)
        elif action == RcRecordSession.CLOSE:
            self._close_bag()

    def _open_bag(self):
        if self._bag is not None:
            return
        try:
            self._bag_path = SequenceAllocator(self._bag_directory, self._bag_prefix).allocate()
            self._bag = rosbag.Bag(self._bag_path, mode='w')
            self._session.bag_opened()
        except Exception as exception:
            self._bag = None
            self._bag_path = ''
            self._session.fail()
            self._publish_status()
            rospy.logfatal('无法创建 bag：%s', exception)
            rospy.signal_shutdown('cannot open bag')
            return
        self._bag_count += 1
        self._bag_opened_at = time.monotonic()
        self._counts = {topic: 0 for topic, _ in self._topics}
        self._want_boost = True
        self._path_publisher.publish(String(data='OPEN ' + self._bag_path))
        rospy.logwarn('开关已拨到触发位：开始录标定包 %s（先静止约 5 s，再绕圈 + 8 字晃动）',
                      self._bag_path)
        self._publish_status()

    def _close_bag(self):
        if self._bag is None:
            return
        path = self._bag_path
        duration = time.monotonic() - self._bag_opened_at
        counts = dict(self._counts)
        close_failed = False
        try:
            self._bag.close()
        except Exception as exception:
            close_failed = True
            rospy.logerr('关闭 %s 失败：%s', path, exception)
        finally:
            self._bag = None
            self._bag_path = ''
            self._want_boost = False
        if close_failed:
            self._session.fail()
            self._path_publisher.publish(String(data='CLOSE_FAILED ' + path))
            self._publish_status()
            return
        self._session.bag_closed()
        self._path_publisher.publish(String(data='CLOSED ' + path))
        rospy.logwarn('开关已离开触发位：已收尾 %s', path)
        self._report(path, counts, duration)
        self._publish_status()

    def _report(self, path, counts, duration):
        rates, problems = bag_health(counts, duration, self._min_fcu_imu_hz)
        lines = ['  %-24s %7d 条  %7.1f Hz' % (topic, counts[topic], rates[topic])
                 for topic, _ in self._topics]
        rospy.loginfo('标定包体检 %s（%.1f s）：\n%s', os.path.basename(path), duration,
                      '\n'.join(lines))
        for problem in problems:
            rospy.logwarn('标定包体检：%s', problem)
        if not problems:
            rospy.loginfo('标定包体检通过：%s', path)

    # ------------------------------------------------------------ 数据 ----
    def _data_callback(self, message, topic):
        with self._lock:
            if self._bag is None or not self._session.recording():
                return
            try:
                self._bag.write(topic, message, t=message_stamp(message))
                self._counts[topic] += 1
            except Exception as exception:
                rospy.logerr('写 bag 失败（%s）：%s', topic, exception)
                path = self._bag_path
                try:
                    self._bag.close()
                except Exception as close_exception:
                    rospy.logerr('顺便关闭 %s 也失败：%s', path, close_exception)
                self._bag = None
                self._bag_path = ''
                self._want_boost = False
                self._session.fail()
                self._publish_status()
                rospy.signal_shutdown('bag write failed')

    # ------------------------------------------------------ 飞控 IMU 提频 ----
    def _state_callback(self, message):
        with self._lock:
            if message.connected and not self._connected and self._want_boost:
                self._reapply = True   # 飞控重启会丢掉提频设置，重连后补发
            self._connected = message.connected

    def _call_set_interval(self, rate_hz):
        try:
            return bool(self._set_interval(message_id=HIGHRES_IMU_ID,
                                           message_rate=rate_hz).success)
        except (rospy.ServiceException, rospy.ROSException):
            return False

    def _rate_worker(self):
        while not self._stop.wait(RATE_WORKER_PERIOD):
            with self._lock:
                want = self._want_boost
                rate = fcu_rate_request(want, self._boost_applied, self._connected,
                                        self._reapply, self._boost_hz)
                self._reapply = False
            if rate is None:
                continue
            ok = self._call_set_interval(rate)
            with self._lock:
                if ok:
                    self._boost_applied = rate > 0.0
                    self._rate_warned = False
                elif rate > 0.0 and want:
                    self._reapply = True   # 下一拍重试
                if ok and rate > 0.0:
                    rospy.loginfo('已请求飞控 IMU 消息 %d @ %.0f Hz', HIGHRES_IMU_ID, rate)
                elif ok:
                    rospy.loginfo('已恢复飞控 IMU 消息 %d 的默认发送频率', HIGHRES_IMU_ID)
                elif not self._rate_warned:
                    rospy.logwarn('调用 %s（%.0f Hz）失败，稍后重试', SET_INTERVAL_SERVICE, rate)
                    self._rate_warned = True

    def _publish_status(self):
        state = 'REC' if self._session.recording() else 'IDLE'
        self._status_publisher.publish(String(data=(
            'state=%s bags=%d last=%s rc_valid=%s healthy=%s fcu_imu_boost=%s'
            % (state, self._bag_count, self._bag_path or '-', self._session.rc_valid,
               self._session.healthy(), 'on' if self._boost_applied else 'off'))))

    # ------------------------------------------------------------ 退出 ----
    def shutdown(self):
        """先关包，录包中被打断时再尽力把飞控 IMU 恢复默认，然后停工作线程。"""
        self._stop.set()
        with self._lock:
            self._close_bag()
            applied = self._boost_applied
        if applied:
            if self._call_set_interval(0.0):
                rospy.loginfo('退出前已恢复飞控 IMU 默认发送频率')
            else:
                rospy.logwarn('退出前没能恢复飞控 IMU 默认频率（MAVROS 可能已先退出）；'
                              '该设置在飞控重启后自动恢复')
        rospy.loginfo('标定录包节点退出，本次共生成 %d 个包，目录 %s',
                      self._bag_count, self._bag_directory)


def main():
    # roslaunch 直接启动本脚本，输出到管道时是块缓冲，日志会憋着不出来；改成行缓冲。
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
    # 自己接管 SIGINT/SIGTERM：先关包、按需恢复飞控 IMU 频率，再关 rospy（服务调用仍可用）。
    rospy.init_node('calib_bag_recorder', disable_signals=True)
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())
    recorder = CalibBagRecorder()
    while not stop.is_set() and not rospy.is_shutdown():
        stop.wait(0.5)
    recorder.shutdown()
    rospy.signal_shutdown('calib_bag_recorder exit')


if __name__ == '__main__':
    main()
