#!/usr/bin/env python3
"""RC 通道开关触发录包节点（可在飞控侧同步记录 ulog）。

触发：订阅 /mavros/rc/in，监视指定 RC 通道（默认 6）。开关拨到「触发位」即开包，
回到非触发位后再保留 release_tail_s 秒尾巴然后关包（同一段动作只会产生一个包）。

与 ulog 的配对：把飞控 SDLOG_MODE 设为 3（while AUX1 > 30%），并把 RC_MAP_AUX1
指向同一个通道，则拨到同一个位置时 ulog 与本节点的 bag 同时开始、同时停止。
注意 PX4 的判据是 manual_control_setpoint.aux1 > 0.3（PWM 归一化 1000->-1、
1500->0、2000->+1），即 PWM 约 >1650us 才算「触发位」；所以本节点默认
active_high=true、阈值 1650us，与之一致。若遥控器上该开关「拨下」输出的是
低电平，需要在遥控器里把该通道反向，或改用另一个方向，否则 ulog 不会开始。

写入方式：直接用 rosbag Python API 写（与板端 orange/vz_step_control 的
offboard_bag_recorder 一致），不 fork rosbag record 进程；bag 名用六位序号，
崩溃最多留空号、不复用编号。

话题：
  订阅  <rc_topic>             mavros_msgs/RCIn         触发源
        每个 ~topics 里的数据话题；~latched_topics 里的话题（只在变化时发布）
        平时也缓存最新一条，开包时先补写当前值
  发布  ~/bag_path (latched)   std_msgs/String          "OPEN <path>" / "CLOSED <path>"
        ~/status                std_msgs/String          状态（开关电平、是否在录）

参数（rospy 私有参数，命令行可直接 _name:=value 覆盖）：
  rc_topic, rc_channel(1-based), rc_threshold, active_high, confirm_samples,
  release_tail_s, rc_warn_timeout_s, stop_on_rc_loss, bag_directory, bag_prefix,
  topics, latched_topics, record_raw_cloud, record_registered_cloud, status_topic, bag_path_topic, status_rate
"""

import os
import sys
import threading
import time

import rosbag
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, TwistStamped,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from mavros_msgs.msg import EstimatorStatus, ExtendedState, PositionTarget, RCIn, State, TimesyncStatus

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rc_record_policy import RcRecordSession  # noqa: E402
from sequence_allocator import SequenceAllocator  # noqa: E402

TICK_PERIOD = 0.2            # 5 Hz 心跳：保证尾巴到期有人关包
DEFAULT_TOPICS = [
    '/mavros/state',
    '/mavros/extended_state',
    '/mavros/estimator_status',
    '/mavros/timesync_status',
    '/mavros/local_position/pose',
    '/mavros/local_position/odom',
    '/mavros/local_position/velocity_local',
    '/mavros/vision_pose/pose_cov',
    '/mavros/odometry/out',
    '/fastlio/vision_odom',
    '/fastlio/odom_world',
    '/tether_winch_control/status',
    '/Odometry',
    '/livox/imu',
    '/front_marker_control/control_mode',
    '/front_marker_control/status',
    '/front_marker_control/position_error',
    '/mavros/setpoint_raw/local',
]
# 只在变化时发布的 latch 话题：开包前收到的最新一条要在开包时补写，否则包里要等下一次变化才有。
DEFAULT_LATCHED_TOPICS = ['/front_marker_control/control_mode']


def topic_type_registry():
    registry = {
        '/mavros/state': State,
        '/mavros/extended_state': ExtendedState,
        '/mavros/estimator_status': EstimatorStatus,
        '/mavros/timesync_status': TimesyncStatus,
        '/mavros/local_position/pose': PoseStamped,
        '/mavros/local_position/odom': Odometry,
        '/mavros/local_position/velocity_local': TwistStamped,
        '/mavros/vision_pose/pose_cov': PoseWithCovarianceStamped,
        '/fastlio/vision_pose_cov': PoseWithCovarianceStamped,
        # ev_mode=odom 时实际送飞控的位姿 + 机体系速度（及本地回显）
        '/mavros/odometry/out': Odometry,
        '/fastlio/vision_odom': Odometry,
        '/fastlio/odom_world': Odometry,
        '/fastlio/velocity': Vector3Stamped,
        '/cloud_registered': PointCloud2,
        '/Odometry': Odometry,
        '/livox/imu': Imu,
        '/tf': TFMessage,
        '/tf_static': TFMessage,
        # front_marker_control：状态机阶段（latch，只在变化时发布）、状态文本、视觉位置误差、实际 setpoint
        '/front_marker_control/control_mode': String,
        '/front_marker_control/status': String,
        '/front_marker_control/position_error': Vector3Stamped,
        '/mavros/setpoint_raw/local': PositionTarget,
    }
    try:
        from tether_winch_control.msg import WinchStatus
        registry['/tether_winch_control/status'] = WinchStatus
    except ImportError:
        rospy.logwarn('无法导入 tether_winch_control/WinchStatus；'
                      '请构建 orange 工作空间并 source orange/devel/setup.bash --extend')
    try:
        from livox_ros_driver2.msg import CustomMsg
        registry['/livox/lidar'] = CustomMsg
    except Exception:
        pass
    try:
        from traj_utils.msg import Bspline
        from drone_navigation.msg import NavigationCommand
        registry.update({
            '/drone_nav/odom': Odometry,
            '/drone_nav/sensor_odom': Odometry,
            '/drone_nav/cloud': PointCloud2,
            '/drone_nav/planner_goal': PoseStamped,
            '/drone_nav/bspline': Bspline,
            '/drone_nav/command': NavigationCommand,
            '/drone_nav/setpoint_preview': PositionTarget,
            '/drone_nav/input_status': String,
            '/drone_nav/planner_status': String,
            '/drone_nav/executor_status': String,
            '/drone_nav/gateway_status': String,
            '/drone_reloc/status': String,
            '/drone_reloc/result': String,
        })
    except ImportError:
        rospy.logwarn('navigation recording types not built; existing recorder topics remain available')
    return registry


def message_stamp(message):
    """bag 时间戳：有 header.stamp 且非零就用它，否则用接收时刻（与 rosbag record 同口径）。"""
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None) if header is not None else None
    if stamp is not None and (stamp.secs or stamp.nsecs):
        return stamp
    return rospy.Time.now()


class RcBagRecorder(object):
    def __init__(self):
        self._lock = threading.RLock()
        self._bag = None
        self._bag_path = ''
        self._bag_count = 0
        self._last_rc_time = None
        self._rc_warned = False
        self._last_logged_pwm = None
        self._last_log_time = 0.0
        self._tick_count = 0

        self._rc_topic = rospy.get_param('~rc_topic', '/mavros/rc/in')
        self._rc_channel = int(rospy.get_param('~rc_channel', 6))
        self._rc_threshold = int(rospy.get_param('~rc_threshold', 1650))
        self._active_high = bool(rospy.get_param('~active_high', True))
        self._rc_warn_timeout = float(rospy.get_param('~rc_warn_timeout_s', 5.0))
        self._stop_on_rc_loss = bool(rospy.get_param('~stop_on_rc_loss', False))
        self._bag_directory = os.path.abspath(
            rospy.get_param('~bag_directory', '/home/orangepi/test_data/bags'))
        self._bag_prefix = rospy.get_param('~bag_prefix', 'handheld')
        self._status_rate = float(rospy.get_param('~status_rate', 2.0))

        self._session = RcRecordSession(
            confirm_samples=int(rospy.get_param('~confirm_samples', 3)),
            release_tail_s=float(rospy.get_param('~release_tail_s', 1.0)))

        registry = topic_type_registry()
        wanted = list(rospy.get_param('~topics', DEFAULT_TOPICS))
        if bool(rospy.get_param('~record_raw_cloud', False)):
            if '/livox/lidar' in registry:
                if '/livox/lidar' not in wanted:
                    wanted.append('/livox/lidar')
            else:
                rospy.logerr('record_raw_cloud=true 但导入不到 livox_ros_driver2/CustomMsg'
                             '（需要 source mid360s_ws/devel/setup.bash），已忽略')
        if bool(rospy.get_param('~record_registered_cloud', False)):
            if '/cloud_registered' not in wanted:
                wanted.append('/cloud_registered')
        self._topics = []
        for topic in wanted:
            if topic not in registry:
                rospy.logerr('话题 %s 没有登记消息类型，已跳过（可用话题：%s）',
                             topic, ', '.join(sorted(registry)))
                continue
            self._topics.append((topic, registry[topic]))
        if not self._topics:
            rospy.logfatal('没有可录制的话题，退出')
            rospy.signal_shutdown('no topics to record')
            return
        recorded = set(t for t, _ in self._topics)
        latched = list(rospy.get_param('~latched_topics', DEFAULT_LATCHED_TOPICS))
        for topic in latched:
            if topic not in recorded:
                rospy.logwarn('latched_topics 里的 %s 不在录制话题里，已忽略', topic)
        self._latched = dict((t, None) for t in latched if t in recorded)

        self._path_publisher = rospy.Publisher(
            rospy.get_param('~bag_path_topic', '/rc_record/bag_path'),
            String, queue_size=1, latch=True)
        self._status_publisher = rospy.Publisher(
            rospy.get_param('~status_topic', '/rc_record/status'),
            String, queue_size=1)

        publishers = dict(rospy.get_published_topics())
        missing = [t for t, _ in self._topics if t not in publishers]
        if missing:
            rospy.logwarn('启动时还没有发布者的话题（稍后出现也没关系）：%s',
                          ', '.join(missing))

        self._subscribers = [
            rospy.Subscriber(topic, message_type, self._data_callback,
                             callback_args=topic, queue_size=500,
                             tcp_nodelay=True)
            for topic, message_type in self._topics
        ]
        self._rc_subscriber = rospy.Subscriber(
            self._rc_topic, RCIn, self._rc_callback, queue_size=20,
            tcp_nodelay=True)
        self._timer = rospy.Timer(rospy.Duration(TICK_PERIOD), self._tick)
        rospy.on_shutdown(self.close)

        rospy.loginfo('RC 触发录包就绪：%s 的通道 %d，阈值 %d us，active_high=%s，'
                      '确认 %d 拍，尾巴 %.1f s，输出目录 %s',
                      self._rc_topic, self._rc_channel, self._rc_threshold,
                      self._active_high, self._session.confirm_samples,
                      self._session.release_tail_s, self._bag_directory)
        rospy.loginfo('录制话题（%d 个）：%s', len(self._topics),
                      ', '.join(t for t, _ in self._topics))
        if self._latched:
            rospy.loginfo('开包时补写当前值的 latch 话题：%s', ', '.join(self._latched))
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
            action = self._session.on_rc(now, True, active)
            self._apply(action, now)

    def _rc_lost(self, now, reason):
        if not self._rc_warned:
            rospy.logwarn('收不到 RC 数据（%s，>%.1fs）；正在录制的包继续录，不会截断',
                          reason, self._rc_warn_timeout)
            self._rc_warned = True
        if self._stop_on_rc_loss:
            self._apply(self._session.note_rc_loss(), now)

    def _tick(self, _event):
        now = time.monotonic()
        with self._lock:
            if self._last_rc_time is not None and \
                    now - self._last_rc_time > self._rc_warn_timeout:
                self._rc_lost(now, '%s 已静默' % self._rc_topic)
            self._apply(self._session.tick(now), now)
            self._tick_count += 1
            if self._tick_count * TICK_PERIOD * self._status_rate >= 1.0:
                self._tick_count = 0
                self._publish_status()

    # ------------------------------------------------------ 状态机动作 ----
    def _apply(self, action, now):
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
            allocator = SequenceAllocator(self._bag_directory, self._bag_prefix)
            self._bag_path = allocator.allocate()
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
        self._write_latched()
        self._path_publisher.publish(String(data='OPEN ' + self._bag_path))
        rospy.logwarn('开关已拨到触发位：开始录包 %s', self._bag_path)
        self._publish_status()

    def _close_bag(self):
        if self._bag is None:
            return
        path = self._bag_path
        close_failed = False
        try:
            self._bag.close()
        except Exception as exception:
            close_failed = True
            rospy.logerr('关闭 %s 失败：%s', path, exception)
        finally:
            self._bag = None
            self._bag_path = ''
        if close_failed:
            self._session.fail()
            self._path_publisher.publish(String(data='CLOSE_FAILED ' + path))
            self._publish_status()
            return
        self._session.bag_closed()
        self._path_publisher.publish(String(data='CLOSED ' + path))
        rospy.logwarn('开关已离开触发位：已收尾 %s', path)
        self._publish_status()

    # ------------------------------------------------------------ 数据 ----
    def _write_latched(self):
        """开包时补写 latch 话题的当前值（时间记为开包时刻）。"""
        now = rospy.Time.now()
        for topic, message in self._latched.items():
            if message is None:
                continue
            try:
                self._bag.write(topic, message, t=now)
            except Exception as exception:
                rospy.logerr('补写 latch 话题 %s 失败：%s', topic, exception)

    def _data_callback(self, message, topic):
        with self._lock:
            if topic in self._latched:
                self._latched[topic] = message
            if self._bag is None or not self._session.recording():
                return
            try:
                self._bag.write(topic, message, t=message_stamp(message))
            except Exception as exception:
                rospy.logerr('写 bag 失败（%s）：%s', topic, exception)
                path = self._bag_path
                try:
                    self._bag.close()
                except Exception as close_exception:
                    rospy.logerr('顺便关闭 %s 也失败：%s', path, close_exception)
                self._bag = None
                self._bag_path = ''
                self._session.fail()
                self._publish_status()
                rospy.signal_shutdown('bag write failed')

    def _publish_status(self):
        state = 'REC' if self._session.recording() else 'IDLE'
        self._status_publisher.publish(String(data=(
            'state=%s bags=%d last=%s rc_valid=%s healthy=%s'
            % (state, self._bag_count, self._bag_path or '-',
               self._session.rc_valid, self._session.healthy()))))

    def close(self):
        with self._lock:
            self._close_bag()
            rospy.loginfo('RC 触发录包节点退出，本次共生成 %d 个包，目录 %s',
                          self._bag_count, self._bag_directory)


def main():
    rospy.init_node('rc_bag_recorder')
    RcBagRecorder()
    rospy.spin()


if __name__ == '__main__':
    main()
