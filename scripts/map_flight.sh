#!/usr/bin/env bash
# ==============================================================================
# 飞行建图（map_flight）：以 fusion.sh 为主体起整条飞行链路，只多两件事——
#   1) 另起一个 C++ rosbag record，从 fusion 起来（飞机在地上静止）一直录到 Ctrl+C：
#      /livox/lidar /livox/imu /tf_static /fastlio/gravity_init /Odometry → ~/test_data/mapping/NAME_raw_<时间>.bag
#   2) fusion 停下后（落地后 Ctrl+C），对这个原始包跑 build_map.py --bag 离线建图，
#      产出 ~/test_data/mapping/<时间>_NAME_raw_<时间>/（scans.pcd、map_world.pcd、preview.png、meta.json 等）。
#   fusion 自己的 RC 触发录包照旧（只录原来那些轻量话题），也放进 ~/test_data/mapping，前缀 NAME_rc。
#
# 路径: /home/orangepi/mid360s_ws/scripts/map_flight.sh
# 用法: ./map_flight.sh [--prefix NAME] [--no-build] [--no-winch] [--no-camera] [--allow-unsynced-clock]
#   --prefix NAME   包名前缀（默认 flight）
#   --no-build      只录包、不建图（之后可以手动 build_map.py --bag 包）
#   其余三个选项原样传给 fusion.sh；-h|--help 看本说明
#
# 为什么这样做（2026-10-18）：
# - 手持建图时人一直在雷达 0.5–1.2 m 内，下压机头扫低处还会让 FAST-LIO 退化；飞着建图没有这两个问题。
# - build_map.py 自己起驱动和 FAST-LIO，不能和 fusion 同时跑；飞行时也不该让 FAST-LIO 开 pcd_save 一直攒点。
#   所以飞行时只录原始数据，落地后离线重建：离线重放和实机逐帧一致（10-18 第二组 1056 帧差 0.000 m）。
# - 原始点云不再让 fusion 的 Python 录包节点录（第一版用 RC_RECORD_RAW_CLOUD）：它按类型订阅，每帧 2 万点要在
#   Python 里反序列化再序列化，A76 大核 69 ms/帧（10 Hz 占大半个核）、A55 小核 292 ms/帧（跟不上），而且从 fusion
#   一起来就一直占着。C++ rosbag record 不解析消息、直接写字节，0.03 ms/帧，fusion 回到以前稳定飞行时的配置。
# - fusion.sh、rc_record 都不改。原始包由后台的独立会话录：终端 Ctrl+C 只到 fusion 和本脚本，fusion 收尾退出后
#   本脚本再 SIGINT 停它；万一本脚本被杀（比如 ssh 断开），它发现 ROS master 连续约 10 s 没了也会自己收尾。
#
# 飞行要点（离线重建靠这几条）：
# - fusion 起来前后飞机都在地上静止：原始包从这时开始录，离线重放时 FAST-LIO 用开头的静止段初始化，
#   原点 = 开始录时的雷达 IMU；看到终端「原始包开始录」再起飞；
# - 机身基本水平，平移 ≤ 0.3 m/s，这个房间 yaw 先 ≤ 15°/s，不要大角度俯仰；
# - 落回起飞点，静止几秒（首尾位姿差 ≈ 漂移），再 Ctrl+C 停 fusion，脚本接着收尾原始包、建图；
# - 离线建图按 1 倍速重放，原始包多长就要再等多久；建图时按 Ctrl+C 会提前收尾（地图只到那一刻）。
# 退出码: 同 fusion.sh（130 = Ctrl+C 正常停）；没录到原始包、包里没有 /livox/lidar 或建图失败时返回 1。
# ==============================================================================
set -o pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/.." && pwd)"
BAG_DIR="/home/orangepi/test_data/mapping"
PREFIX="flight"
DO_BUILD=1
FUSION_ARGS=()
RAW_TOPICS=(/livox/lidar /livox/imu /tf_static /fastlio/gravity_init /Odometry)
REC_BIN=/opt/ros/noetic/lib/rosbag/record
MASTER_URI="http://192.168.31.32:11311"   # 同 fusion.sh
ROS_IP_ADDR="192.168.31.32"

while [ $# -gt 0 ]; do
  case "$1" in
    --prefix) PREFIX="${2:-}"; shift ;;
    --no-build) DO_BUILD=0 ;;
    --no-winch|--no-camera|--allow-unsynced-clock) FUSION_ARGS+=("$1") ;;
    --no-record|--bag-dir|--bag-prefix)
      echo "$1 不能用：本脚本就是为了录包，目录固定为 $BAG_DIR，前缀用 --prefix" >&2; exit 2 ;;
    -h|--help) sed -n '3,35p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
  shift
done
[[ "$PREFIX" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "--prefix 只能用字母、数字、_、-" >&2; exit 2; }
for f in "$DIR/fusion.sh" "$DIR/build_map.py" "$REC_BIN"; do
  [ -x "$f" ] || { echo "缺少 $f" >&2; exit 1; }
done
mkdir -p "$BAG_DIR" || exit 1
# 已经有 fusion（或 start_handheld_test.sh）在跑时，里面的 fusion.sh 会因为锁马上退出，但录包进程可能已经
# 把那条在跑的链路录进来了。所以先探一下 fusion 的锁：拿得到就马上放掉，交给 fusion.sh 自己去抢
FUSION_LOCK=/tmp/mid360s_fusion.lock
if ! (exec 9>>"$FUSION_LOCK" && flock -n 9) 2>/dev/null; then
  echo "已有 fusion 实例在跑（$FUSION_LOCK 被占用：$(tail -n 1 "$FUSION_LOCK" 2>/dev/null)），先停掉它" >&2
  exit 1
fi
RUN_DIR="$(mktemp -d /tmp/map_flight.XXXXXX)" || exit 1
trap 'rm -rf "$RUN_DIR"' EXIT

log() { printf '[%s] [map_flight] %s\n' "$(date +%T)" "$*"; }

# ------------------------------------------------------------- 原始包录制 ----
# 独立会话里的后台进程：等 master 起来、/livox/lidar 有发布者，按当时（fusion 已对上时钟）的时间起名，
# 把包路径写进 RUN_DIR/raw.path 再开录。收到 TERM（本脚本收尾）或 master 连续约 10 s 不在时 SIGINT 停录包。
# 非交互 shell 的后台任务继承「忽略 SIGINT」，所以这里只认 TERM；rosbag record 自己装 SIGINT 处理，不受影响。
start_raw_recorder() {
  ROS_MASTER_URI="$MASTER_URI" ROS_IP="$ROS_IP_ADDR" setsid bash -c '
    run_dir="$1"; bag_dir="$2"; prefix="$3"; ws="$4"; rec="$5"; shift 5
    set +u; source /opt/ros/noetic/setup.bash; source "$ws/devel/setup.bash"; unset ROS_HOSTNAME
    stop=0; trap "stop=1" TERM
    master_up() { (exec 3<>/dev/tcp/192.168.31.32/11311) 2>/dev/null; }
    while [ "$stop" -eq 0 ]; do
      if master_up && timeout 4 rostopic info /livox/lidar 2>/dev/null | sed -n "/^Publishers:/,/^Subscribers:/p" | grep -q "^ \* "; then
        break
      fi
      sleep 1
    done
    [ "$stop" -eq 1 ] && { echo "还没开录就收到停止"; exit 0; }
    bag="$bag_dir/${prefix}_raw_$(date +%Y%m%d_%H%M%S).bag"
    echo "$bag" > "$run_dir/raw.path"
    "$rec" -O "$bag" "$@" &
    rpid=$!
    echo "原始包开始录：$bag（record pid $rpid）"
    lost=0
    while [ "$stop" -eq 0 ] && kill -0 "$rpid" 2>/dev/null; do
      if master_up; then lost=0; else lost=$((lost + 1)); fi
      [ "$lost" -ge 5 ] && { echo "ROS master 约 10 s 不在了，停录包"; break; }
      sleep 2
    done
    kill -INT "$rpid" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 "$rpid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$rpid" 2>/dev/null && { echo "rosbag record 20 s 没退，SIGTERM"; kill -TERM "$rpid"; }
    wait "$rpid" 2>/dev/null
    sync -f "$bag_dir"
    echo "原始包已收尾"
  ' _ "$RUN_DIR" "$BAG_DIR" "$PREFIX" "$WS" "$REC_BIN" "${RAW_TOPICS[@]}" \
    </dev/null >"$RUN_DIR/recorder.log" 2>&1 &
  REC_HELPER=$!
}

stop_raw_recorder() {
  kill -0 "$REC_HELPER" 2>/dev/null || return 0
  kill -TERM "$REC_HELPER" 2>/dev/null
  for _ in $(seq 1 60); do kill -0 "$REC_HELPER" 2>/dev/null || return 0; sleep 0.5; done
  log "[WARN] 原始包录制进程 30 s 没退出（pid $REC_HELPER），请手动检查"
  return 1
}

# SIGINT 只记一下，不能 trap ''：忽略会被子进程 fusion.sh 继承下去，它的 Ctrl+C 收尾就失效了。
# fusion.sh 在前台跑，终端的 Ctrl+C 会同时到它和本脚本；本脚本的 trap 等它收尾退出后才执行。
INTR=0
trap 'INTR=1' INT

log "原始包：C++ rosbag record 录 ${RAW_TOPICS[*]} → $BAG_DIR/${PREFIX}_raw_<时间>.bag"
log "fusion 的 RC 录包照旧，放 $BAG_DIR，前缀 ${PREFIX}_rc；fusion 停下后$([ "$DO_BUILD" -eq 1 ] && echo '自动离线建图' || echo '不建图')"
log "飞机在地上静止，等终端出现「原始包开始录」再起飞；落地静止几秒后 Ctrl+C"
start_raw_recorder
# 录包进程的输出实时打到终端；--pid 让 tail 在录包进程退出后自己结束，不留孤儿进程。
# 滤掉 rosbag record 的 INFO（有输出缓冲，退出时才一起出来）和 master 停掉后刷的 XmlRpc 连接报错（收尾时正常）
tail -n +1 -F --pid="$REC_HELPER" "$RUN_DIR/recorder.log" 2>/dev/null \
  | grep --line-buffered -v -e 'XmlRpc' -e '\[ *INFO\]' | sed -u 's/^/[map_flight] /' &
TAIL_PID=$!
"$DIR/fusion.sh" --bag-dir "$BAG_DIR" --bag-prefix "${PREFIX}_rc" "${FUSION_ARGS[@]}"
RC=$?
INTR=0
log "fusion 已退出（$RC），收尾原始包"
stop_raw_recorder
wait "$TAIL_PID" 2>/dev/null

RAW="$(cat "$RUN_DIR/raw.path" 2>/dev/null)"
if [ -z "$RAW" ] || [ ! -f "$RAW" ]; then
  [ -n "$RAW" ] && [ -f "$RAW.active" ] && log "[WARN] 只有 $RAW.active（没正常收尾），可以 rosbag reindex 后再 build_map.py --bag"
  log "没录到原始包（fusion 没起来，或者 /livox/lidar 一直没有发布者），不建图"
  exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
if ! rosbag info "$RAW" 2>/dev/null | grep -q '/livox/lidar '; then
  log "[WARN] $(basename "$RAW") 里没有 /livox/lidar，不建图"
  exit 1
fi
log "原始包 $(basename "$RAW")：$(du -h "$RAW" | cut -f1)，$(rosbag info --yaml --key=duration "$RAW" 2>/dev/null | cut -d. -f1) s"
if [ "$DO_BUILD" -eq 0 ]; then
  log "--no-build：不建图。之后可以：$DIR/build_map.py --bag $RAW"
  exit "$RC"
fi
log "离线建图（1 倍速重放，约等于原始包时长）"
if ! "$DIR/build_map.py" --bag "$RAW" --name "$(basename "$RAW" .bag)"; then
  log "建图没完成，看上面的输出；原始包在 $RAW"
  exit 1
fi
log "完成：地图在 $BAG_DIR/<时间>_$(basename "$RAW" .bag)/，原始包 $RAW"
exit "$RC"
