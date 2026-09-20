#!/usr/bin/env bash
# 端到端验证「RC 通道电平 -> 开包/关包」（隔离 Master + 假 RC/位姿发布器，不需要真遥控器与飞控）
#   ./test_rc_record_e2e.sh [master 端口，默认 11377]
# 期望：脚本自动翻转通道 6 两次触发位，最终在临时目录里得到 2 个有效 bag。
set -u
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-11377}"
export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="127.0.0.1"
set +u; source /opt/ros/noetic/setup.bash; set -u

BAGDIR="$(mktemp -d /tmp/rc_record_e2e.XXXXXX)"
CORE_PID=""; NODE_PID=""; PUB_PID=""
cleanup() {
  [ -n "$PUB_PID" ] && kill -INT "$PUB_PID" 2>/dev/null
  [ -n "$NODE_PID" ] && kill -INT "$NODE_PID" 2>/dev/null
  sleep 2
  [ -n "$CORE_PID" ] && kill -INT "$CORE_PID" 2>/dev/null
  sleep 2
  for p in $PUB_PID $NODE_PID $CORE_PID; do kill -9 "$p" 2>/dev/null; done
  rm -f /tmp/rc_record_e2e_state.$$ 2>/dev/null
}
trap cleanup EXIT

echo "== 隔离 Master 127.0.0.1:$PORT，bag 目录 $BAGDIR =="
roscore -p "$PORT" >"$BAGDIR/roscore.log" 2>&1 &
CORE_PID=$!
sleep 4

sed "s|__BAGDIR__|$BAGDIR|" >"$BAGDIR/config.yaml" <<'YAML'
rc_bag_recorder:
  bag_directory: __BAGDIR__
  bag_prefix: e2e
  rc_topic: /test/rc_in
  rc_channel: 6
  rc_threshold: 1650
  active_high: true
  confirm_samples: 2
  release_tail_s: 1.0
  topics: [/mavros/local_position/pose]
YAML
rosparam load "$BAGDIR/config.yaml" || { echo "[FAIL] rosparam load 失败"; exit 1; }

python3 -u "$DIR/bag_recorder_node.py" __name:=rc_bag_recorder >"$BAGDIR/node.log" 2>&1 &
NODE_PID=$!
sleep 3
if ! pgrep -f "bag_recorder_node.p[y]" >/dev/null 2>&1; then
  echo "[FAIL] 节点未启动，日志："; tail -20 "$BAGDIR/node.log"; exit 1
fi

python3 - "$BAGDIR" <<'PYEOF' >"$BAGDIR/publisher.log" 2>&1 &
import sys, time, rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn

rospy.init_node('fake_rc_and_pose', anonymous=True, disable_signals=True)
bagdir = sys.argv[1]
pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)
rc_pub = rospy.Publisher('/test/rc_in', RCIn, queue_size=10)
time.sleep(1.0)   # 等订阅者连上

script = [(1000, 2.0), (2000, 3.0), (1000, 2.5), (2000, 2.0), (1000, 1.5)]
start = time.monotonic()
pose_count = 0
for pwm, duration in script:
    end = time.monotonic() + duration
    while time.monotonic() < end and not rospy.is_shutdown():
        msg = RCIn()
        channels = [1500] * 8
        channels[5] = pwm        # 通道 6
        msg.channels = channels
        msg.rssi = 200
        rc_pub.publish(msg)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.z = 0.63
        pose_pub.publish(pose)
        pose_count += 1
        print('t=%.2f ch6=%d pose_msgs=%d' % (time.monotonic() - start, pwm, pose_count), flush=True)
        time.sleep(0.05)         # 20 Hz
print('publisher done, pose_msgs=%d' % pose_count, flush=True)
PYEOF
PUB_PID=$!

WAIT=0
while kill -0 "$PUB_PID" 2>/dev/null && [ "$WAIT" -lt 30 ]; do sleep 1; WAIT=$((WAIT+1)); done
echo "== 发布器结束，等待尾巴关包 =="
sleep 3

LATEST=$(timeout 5 rostopic echo -n1 /rc_record/bag_path 2>/dev/null | sed -n 's/^data: "//p' | tr -d '"')
echo "== /rc_record/bag_path（latched，节点仍在运行）= ${LATEST:-无}"

# 先让节点正常退出（顺带验证 SIGINT 收尾），再读它无缓冲的日志
if [ -n "${NODE_PID:-}" ]; then kill -INT "$NODE_PID" 2>/dev/null; sleep 2; fi
echo "== 节点日志（关键行）=="
grep -aE "就绪|开始录包|已收尾|WARN|ERROR|FAIL|触发" "$BAGDIR/node.log" | tail -18

echo "== bag 结果 =="
ls -1 "$BAGDIR"/e2e_*.bag 2>/dev/null || echo "（没有生成任何 bag）"
ACTIVE=$(ls -1 "$BAGDIR"/e2e_*.bag.active 2>/dev/null | wc -l)
BAGS=$(ls -1 "$BAGDIR"/e2e_*.bag 2>/dev/null | wc -l)
FAILED=0
[ "$BAGS" -eq 2 ] || { echo "[FAIL] 期望 2 个 bag，实际 $BAGS"; FAILED=1; }
[ "$ACTIVE" -eq 0 ] || { echo "[FAIL] 还有未收尾的 .bag.active"; FAILED=1; }
for b in "$BAGDIR"/e2e_*.bag; do
  [ -e "$b" ] || continue
  echo "--- $(basename "$b")"
  rosbag info "$b" | sed -n '/topics:/,$p'
  n=$(rosbag info "$b" | awk '/\/mavros\/local_position\/pose/{print $3}')
  [ "${n:-0}" -gt 10 ] || { echo "[FAIL] $b 里 pose 消息太少（$n）"; FAILED=1; }
done
echo
if [ "$FAILED" -eq 0 ]; then echo "=== E2E 通过 ==="; else echo "=== E2E 失败 ==="; fi
echo "（bag 留在 $BAGDIR，便于人工检查）"
exit "$FAILED"
