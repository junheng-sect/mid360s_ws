#!/usr/bin/env bash
# 端到端验证标定录包节点（隔离 Master + 假数据，不需要真飞控、遥控器和雷达）：
#   ./calib_record_e2e.sh [master 端口，默认 11379]
# 假发布器：/mavros/state(connected) + RC 通道 6 翻转一次 + 200 Hz 两路 IMU + 10 Hz CustomMsg
#           + 10 Hz timesync + 干扰话题 /Odometry；另起假的 /mavros/set_message_interval 服务记录调用。
# 期望：只生成 1 个包、只含 4 个标定话题（/Odometry 不进包）；服务依次收到 400 与 0；
#       节点日志里有关包体检摘要。
set -u
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$DIR/.." && pwd)"
PORT="${1:-11379}"
export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="127.0.0.1"
set +u
source /opt/ros/noetic/setup.bash
source /home/orangepi/mid360s_ws/devel/setup.bash
set -u

WORK="$(mktemp -d /tmp/calib_record_e2e.XXXXXX)"
CORE_PID=""; NODE_PID=""
cleanup() {
  [ -n "$NODE_PID" ] && kill -INT "$NODE_PID" 2>/dev/null
  sleep 2
  [ -n "$CORE_PID" ] && kill -INT "$CORE_PID" 2>/dev/null
  sleep 2
  for p in $NODE_PID $CORE_PID; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT

echo "== 隔离 Master 127.0.0.1:$PORT，工作目录 $WORK =="
roscore -p "$PORT" >"$WORK/roscore.log" 2>&1 &
CORE_PID=$!
sleep 4

rosparam load "$PKG/config/calib_record.yaml" /calib_bag_recorder || { echo "[FAIL] rosparam load"; exit 1; }
rosparam set /calib_bag_recorder/bag_directory "$WORK"
rosparam set /calib_bag_recorder/confirm_samples 2
python3 "$PKG/scripts/calib_bag_recorder.py" __name:=calib_bag_recorder >"$WORK/node.log" 2>&1 &
NODE_PID=$!
sleep 3
kill -0 "$NODE_PID" 2>/dev/null || { echo "[FAIL] 节点未启动："; tail -20 "$WORK/node.log"; exit 1; }

python3 - "$WORK/service_calls.txt" <<'PYEOF' >"$WORK/publisher.log" 2>&1
import sys, threading, time
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn, State, TimesyncStatus
from mavros_msgs.srv import MessageInterval, MessageIntervalResponse
from livox_ros_driver2.msg import CustomMsg, CustomPoint

rospy.init_node('fake_calib_sources', disable_signals=True)
calls = []
lock = threading.Lock()


def handle(request):
    with lock:
        calls.append((request.message_id, request.message_rate))
    return MessageIntervalResponse(success=True)


rospy.Service('/mavros/set_message_interval', MessageInterval, handle)
pubs = {
    'state': rospy.Publisher('/mavros/state', State, queue_size=10),
    'rc': rospy.Publisher('/mavros/rc/in', RCIn, queue_size=10),
    'limu': rospy.Publisher('/livox/imu', Imu, queue_size=200),
    'fimu': rospy.Publisher('/mavros/imu/data_raw', Imu, queue_size=200),
    'lidar': rospy.Publisher('/livox/lidar', CustomMsg, queue_size=10),
    'ts': rospy.Publisher('/mavros/timesync_status', TimesyncStatus, queue_size=10),
    'odom': rospy.Publisher('/Odometry', Odometry, queue_size=10),   # 干扰：不应进标定包
}
time.sleep(1.5)
k = 0
for pwm, duration in [(1000, 2.0), (2000, 5.0), (1000, 3.0)]:
    end = time.monotonic() + duration
    while time.monotonic() < end:
        now = rospy.Time.now()
        for key in ('limu', 'fimu'):
            imu = Imu()
            imu.header.stamp = now
            pubs[key].publish(imu)
        if k % 20 == 0:
            pubs['state'].publish(State(connected=True))
        if k % 10 == 0:
            rc = RCIn()
            channels = [1500] * 8
            channels[5] = pwm
            rc.channels = channels
            pubs['rc'].publish(rc)
            cloud = CustomMsg()
            cloud.header.stamp = now
            cloud.point_num = 3
            cloud.points = [CustomPoint(offset_time=i, x=1.0 + i, z=0.5) for i in range(3)]
            pubs['lidar'].publish(cloud)
            ts = TimesyncStatus()
            ts.header.stamp = now
            pubs['ts'].publish(ts)
            odom = Odometry()
            odom.header.stamp = now
            pubs['odom'].publish(odom)
        k += 1
        time.sleep(0.005)
time.sleep(1.0)
with lock, open(sys.argv[1], 'w') as stream:
    for message_id, rate in calls:
        stream.write('%d %.1f\n' % (message_id, rate))
print('publisher done, loops=%d calls=%s' % (k, calls))
PYEOF

kill -INT "$NODE_PID" 2>/dev/null; sleep 2
echo "== 节点日志（关键行）=="
grep -aE "就绪|开始录标定包|已收尾|体检|已请求|已恢复|WARN|ERROR|FATAL" "$WORK/node.log" | tail -20
echo "== 假服务收到的调用 =="
cat "$WORK/service_calls.txt" 2>/dev/null || echo "（无）"

FAILED=0
python3 - "$WORK" <<'CHECK' || FAILED=1
import glob, os, sys
import rosbag
work = sys.argv[1]
bags = sorted(glob.glob(os.path.join(work, 'calib_*.bag')))
assert len(bags) == 1, bags
with rosbag.Bag(bags[0]) as bag:
    topics = {topic: info.msg_type for topic, info in bag.get_type_and_topic_info().topics.items()}
    counts = {topic: info.message_count for topic, info in bag.get_type_and_topic_info().topics.items()}
expected = {'/livox/lidar': 'livox_ros_driver2/CustomMsg', '/livox/imu': 'sensor_msgs/Imu',
            '/mavros/imu/data_raw': 'sensor_msgs/Imu',
            '/mavros/timesync_status': 'mavros_msgs/TimesyncStatus'}
assert topics == expected, topics
assert all(count > 10 for count in counts.values()), counts
with open(os.path.join(work, 'service_calls.txt')) as stream:
    calls = [line.split() for line in stream if line.strip()]
assert [(int(i), float(r)) for i, r in calls] == [(105, 400.0), (105, 0.0)], calls
with open(os.path.join(work, 'node.log'), 'rb') as stream:
    log = stream.read().decode('utf-8', 'replace')
assert '标定包体检' in log, 'no health report in node log'
print('[PASS] %s：只含 4 个标定话题 %s；服务调用 400 -> 0；有体检摘要' % (bags[0], counts))
CHECK
echo
if [ "$FAILED" -eq 0 ]; then echo "=== 标定录包 E2E 通过 ==="; else echo "=== 标定录包 E2E 失败 ==="; fi
echo "（结果留在 $WORK）"
exit "$FAILED"
