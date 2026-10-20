#!/usr/bin/env bash
# 飞行建图：fusion 飞行链路 + CH6 控制的 C++ 原始点云录包，退出后离线建图。
# 用法: ./map_flight.sh [--prefix NAME] [--no-build] [--no-winch] [--no-camera] [--allow-unsynced-clock]
# CH6 开：同时开启轻量包与 C++ 点云包；CH6 关：按原有 1 s 尾巴一起收尾。
# 飞控 ULog 沿用原来的 CH6 / AUX1 配置；本脚本不改飞控参数。
# 点云包 NAME_raw_000001.bag 与轻量包 NAME_rc_000001.bag 共用序号。
# 原始包录 /livox/lidar /livox/imu /tf_static /fastlio/gravity_init /Odometry。
# 每次开录时飞机在地面静止 5–10 s，供该包离线初始化；落地上锁、CH6 关后再 Ctrl+C。
# 退出后对每个原始包调 drone_mapping 的 build_map --bag（离线 Voxel-SLAM，耗时约等于包长）。
# Python 控制器只监听轻量开/关包事件和状态，不解析点云。
# C++ 仍有网络/缓冲/磁盘开销，改录包方式不代表此前飞行异常已经解决。
# --no-build 只录包；其余三个开关传给 fusion。原始包在 ~/test_data/mapping。
set -o pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/.." && pwd)"
BUILD_MAP="$WS/src/drone_mapping/scripts/build_map"   # = rosrun drone_mapping build_map
BAG_DIR="/home/orangepi/test_data/mapping"
PREFIX="flight"
DO_BUILD=1
FUSION_ARGS=()
REC_BIN=/opt/ros/noetic/lib/rosbag/record
RAW_HELPER="$DIR/rc_record/raw_bag_recorder.py"
MASTER_URI="http://192.168.31.32:11311"
ROS_IP_ADDR="192.168.31.32"

while [ $# -gt 0 ]; do
  case "$1" in
    --prefix) [ $# -ge 2 ] || { echo '--prefix 缺少名称' >&2; exit 2; }; PREFIX="$2"; shift ;;
    --no-build) DO_BUILD=0 ;;
    --no-winch|--no-camera|--allow-unsynced-clock) FUSION_ARGS+=("$1") ;;
    --no-record|--bag-dir|--bag-prefix)
      echo "$1 不能用：目录固定为 $BAG_DIR，前缀用 --prefix" >&2; exit 2 ;;
    -h|--help) sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
  shift
done
[[ "$PREFIX" =~ ^[A-Za-z0-9_-]+$ ]] || { echo '--prefix 只能用字母、数字、_、-' >&2; exit 2; }
for f in "$DIR/fusion.sh" "$BUILD_MAP" "$REC_BIN"; do
  [ -x "$f" ] || { echo "缺少 $f" >&2; exit 1; }
done
[ -f "$RAW_HELPER" ] || { echo "缺少 $RAW_HELPER" >&2; exit 1; }
mkdir -p "$BAG_DIR" "$WS/logs/fusion" || exit 1
FUSION_LOCK=/tmp/mid360s_fusion.lock
if ! (exec 9>>"$FUSION_LOCK" && flock -n 9) 2>/dev/null; then
  echo "已有 fusion 实例在跑（$(tail -n 1 "$FUSION_LOCK" 2>/dev/null)），先停掉它" >&2
  exit 1
fi
RUN_DIR="$(mktemp -d /tmp/map_flight.XXXXXX)" || exit 1
REC_LOG="$WS/logs/fusion/map_flight_$(date +%Y%m%d_%H%M%S)_$$.log"
REC_HELPER=''
REC_STOPPED=0
REC_RC=0
TAIL_PID=''
log() { printf '[%s] [map_flight] %s\n' "$(date +%T)" "$*"; }
proc_starttime() {
  local value
  value="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  value="${value##*)}"
  set -- $value
  [ "$#" -ge 20 ] || return 1
  printf '%s\n' "${20}"
}
process_running() {
  local value
  value="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  value="${value##*)}"
  set -- $value
  [ "${1:-Z}" != Z ] && [ "${1:-X}" != X ]
}
stop_raw_recorder() {
  [ "$REC_STOPPED" -eq 1 ] && return "$REC_RC"
  REC_STOPPED=1
  local pid='' start='' current
  # The helper registers its actual PID even if setsid forks. Bound the startup race.
  for _ in $(seq 1 100); do
    if read -r pid start 2>/dev/null <"$RUN_DIR/raw.pid" &&
        [[ "$pid" =~ ^[0-9]+$ && "$start" =~ ^[0-9]+$ ]]; then
      break
    fi
    [ -n "$REC_HELPER" ] && process_running "$REC_HELPER" || break
    sleep 0.1
  done
  if [[ "$pid" =~ ^[0-9]+$ && "$start" =~ ^[0-9]+$ ]]; then
    current="$(proc_starttime "$pid")"
    if [ "$current" = "$start" ] && process_running "$pid"; then
      kill -TERM "$pid" 2>/dev/null
    fi
  fi
  # Never enter an unbounded wait, including before the identity file appears.
  for _ in $(seq 1 100); do
    [ -n "$REC_HELPER" ] && process_running "$REC_HELPER" || break
    sleep 0.5
  done
  if [ -n "$REC_HELPER" ] && process_running "$REC_HELPER"; then
    log "[ERROR] 原始录包控制未收尾，不执行建图；日志 $REC_LOG，状态 $RUN_DIR"
    REC_RC=1
    return 1
  fi
  if [ -n "$REC_HELPER" ]; then
    wait "$REC_HELPER" 2>/dev/null
    REC_RC=$?
  fi
  return "$REC_RC"
}
cleanup() {
  stop_raw_recorder
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null
  [ "$REC_RC" -ne 0 ] || rm -rf "$RUN_DIR"
}
trap cleanup EXIT
# Keep SIGINT catchable in fusion; ignoring it here would be inherited.
trap ':' INT

log "CH6 控制所有录包：${PREFIX}_rc_<序号>.bag + ${PREFIX}_raw_<同序号>.bag"
log "开关先置关闭位；链路起来后拨 CH6 开，在地面静止录 5–10 s 再起飞"
log "落地上锁后 CH6 关，等原始包收尾，再 Ctrl+C；日志 $REC_LOG"
ROS_MASTER_URI="$MASTER_URI" ROS_IP="$ROS_IP_ADDR" setsid --wait bash -c '
  ws="$1"; shift
  set +u; source /opt/ros/noetic/setup.bash; source "$ws/devel/setup.bash"; unset ROS_HOSTNAME
  exec python3 -u "$@"
' _ "$WS" "$RAW_HELPER" --run-dir "$RUN_DIR" --bag-dir "$BAG_DIR" --prefix "$PREFIX" --record-bin "$REC_BIN" \
  </dev/null >"$REC_LOG" 2>&1 &
REC_HELPER=$!
tail -n +1 -F --pid="$REC_HELPER" "$REC_LOG" 2>/dev/null \
  | grep --line-buffered -v -e 'XmlRpc' -e '\[ *INFO\]' | sed -u 's/^/[map_flight] /' &
TAIL_PID=$!
# Never re-enable Python raw-cloud deserialization via an inherited environment.
RC_RECORD_RAW_CLOUD=false "$DIR/fusion.sh" --bag-dir "$BAG_DIR" --bag-prefix "${PREFIX}_rc" "${FUSION_ARGS[@]}"
RC=$?
log "fusion 已退出（$RC），收尾 CH6 原始录包"
stop_raw_recorder
FAILED=0
if [ ! -f "$RUN_DIR/raw.summary.json" ]; then
  log "[ERROR] 原始录包没有完整收尾状态，保留日志和 $RUN_DIR，不执行建图"
  exit 1
fi
if [ "$REC_RC" -ne 0 ] || [ ! -f "$RUN_DIR/raw.summary.json" ]; then
  log "[WARN] 原始录包控制未正常完成，检查 $REC_LOG"
  FAILED=1
fi
RAW_BAGS=()
[ ! -f "$RUN_DIR/raw.paths" ] || mapfile -t RAW_BAGS <"$RUN_DIR/raw.paths"
if [ "${#RAW_BAGS[@]}" -eq 0 ]; then
  log '本次没有原始包（CH6 没有开启录包），不建图'
  [ "$FAILED" -eq 0 ] || exit 1
  exit "$RC"
fi
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
for RAW in "${RAW_BAGS[@]}"; do
  if [ ! -f "$RAW" ]; then
    log "[WARN] 未收尾原始包 $RAW（如有 .active，先恢复索引），跳过"
    FAILED=1
    continue
  fi
  INFO="$(rosbag info "$RAW" 2>/dev/null)"
  if [ $? -ne 0 ] || ! grep -q '/livox/lidar ' <<<"$INFO" || ! grep -q '/livox/imu ' <<<"$INFO"; then
    log "[WARN] $(basename "$RAW") 索引无效或缺少 lidar/imu，跳过建图"
    FAILED=1
    continue
  fi
  log "原始包 $(basename "$RAW")：$(du -h "$RAW" | cut -f1)"
  if [ "$DO_BUILD" -eq 0 ]; then
    log "--no-build：保留原始包。之后可以：rosrun drone_mapping build_map --bag $RAW"
  elif ! "$BUILD_MAP" --bag "$RAW" --name "$(basename "$RAW" .bag)"; then
    log "建图没完成；原始包在 $RAW"
    FAILED=1
  fi
done
[ "$FAILED" -eq 0 ] || exit 1
log "完成：本次 ${#RAW_BAGS[@]} 段原始包，输出目录 $BAG_DIR"
exit "$RC"
