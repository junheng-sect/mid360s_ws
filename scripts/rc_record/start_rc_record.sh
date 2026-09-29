#!/usr/bin/env bash
# 启动 RC 开关触发录包节点（单独使用；一键脚本 start_handheld_test.sh 也会调用它）
# 路径: /home/orangepi/mid360s_ws/scripts/rc_record/start_rc_record.sh
# 用法: ./start_rc_record.sh [额外 ROS 参数，例如 __name:=rc_bag_recorder]
# 配置: rc_record.yaml（可用 RC_RECORD_CONFIG 指定别的文件）
# 可被环境变量覆盖：RC_BAG_PREFIX / RC_BAG_DIR / RC_RECORD_RAW_CLOUD
set -o pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${RC_RECORD_CONFIG:-$DIR/rc_record.yaml}"
WS="$(cd "$DIR/../.." && pwd)"

set +u
source /opt/ros/noetic/setup.bash
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"   # /livox/lidar 的 CustomMsg 需要
# Extend rather than replace the MID360S environment: keep Livox messages available.
ORANGE_SETUP="/home/orangepi/orange/devel/setup.bash"
[ -f "$ORANGE_SETUP" ] && source "$ORANGE_SETUP" --extend
set -u

command -v rosparam >/dev/null 2>&1 || { echo "[FAIL] 找不到 rosparam（ROS 环境未就绪）" >&2; exit 1; }
[ -f "$CONFIG" ] || { echo "[FAIL] 配置文件不存在：$CONFIG" >&2; exit 1; }
timeout 5 rosparam list >/dev/null 2>&1 || { echo "[FAIL] 连不上 ROS Master，请先启动 MAVROS" >&2; exit 1; }

rosparam load "$CONFIG"
[ -n "${RC_BAG_PREFIX:-}" ] && rosparam set /rc_bag_recorder/bag_prefix "$RC_BAG_PREFIX"
[ -n "${RC_BAG_DIR:-}" ]    && rosparam set /rc_bag_recorder/bag_directory "$RC_BAG_DIR"
[ -n "${RC_RECORD_RAW_CLOUD:-}" ] && rosparam set /rc_bag_recorder/record_raw_cloud "$RC_RECORD_RAW_CLOUD"
echo "[INFO] 已加载 $CONFIG（prefix=${RC_BAG_PREFIX:-来自yaml}，dir=${RC_BAG_DIR:-来自yaml}），启动 rc_bag_recorder"
exec python3 -u "$DIR/bag_recorder_node.py" "$@"
