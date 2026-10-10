#!/usr/bin/env bash
# ==============================================================================
# 融合快速启动（fusion）：一条 roslaunch 拉起整条链路
#   MAVROS(飞控串口) + MID-360S 驱动 + FAST-LIO2 + 重力对齐桥接(位姿送飞控)
#   + RC 开关触发录包节点（遥控器通道 6 拨到触发位即开始录 bag）
#   + 系留绕线器自动控制（默认开启，非系留使用 --no-winch）
#   + OV13855 相机预览（独立的第二个 roslaunch，见 launch/camera.launch）
#
# 路径: /home/orangepi/mid360s_ws/scripts/fusion.sh
# launch: /home/orangepi/mid360s_ws/scripts/launch/fusion.launch
# 用法: ./fusion.sh [选项]
#   --no-record         不启动 RC 触发录包节点（本次不会有 bag）
#   --no-winch          不启动绕线器控制（非系留测试）
#   --no-camera         不启动相机预览（OV13855 驱动 + AprilTag 视觉节点）
#   --bag-prefix NAME   包名前缀（默认 noprop，产出 NAME_000001.bag）
#   --bag-dir PATH      包输出目录（默认 /home/orangepi/bags）
#   -h|--help           帮助
#
# 前置检查只做三件事：环境文件（ROS/工作空间 + rospack 找包）、飞控串口权限、
# 防重复启动（锁 + 残留进程/节点）。不 ping 雷达、不拉取核对飞控参数、不逐话题
# 测频、不等 EKF 就绪 —— 要逐项体检请用同目录 start_handheld_test.sh；两个入口
# 共用 /tmp/mid360s_fusion.lock，互相排斥。
#
# 手动启动（不经本脚本，launch 里的注释指的就是这段）：
#   cd /home/orangepi/mid360s_ws
#   source /opt/ros/noetic/setup.bash && source devel/setup.bash
#   source /home/orangepi/orange/devel/setup.bash --extend
#   export ROS_PACKAGE_PATH="$PWD/scripts:$ROS_PACKAGE_PATH"   # rc_record 靠它定位
#   roslaunch --screen --required scripts/launch/fusion.launch
#
# 前台运行。roslaunch 被放进独立会话，信号只由本脚本统一转发：Ctrl+C / SIGTERM
# 都只发一次 SIGINT 给本次启动的 roslaunch，由它按顺序收拢全部节点（录包节点收到
# SIGINT 会把 bag 正常收尾写索引）。所有等待都是短轮询（≤ 3 s 一轮）并顺带检查
# roslaunch 是否还活着，所以启动/等首帧/稳态各阶段收到信号都能及时收尾。
# 收尾后校验本次生成的 bag 索引是否读得出来，最后按信号返回 130 / 143；roslaunch
# 自己退出时透传它的退出码，并在「没收到停止信号」时按异常返回 1（补掉 roslaunch
# 对 required 节点退出也返回 0 的坑）。本脚本不改飞控参数、不改源码、不做开机自启；
# 信号处理与退出码按后续接入 systemd 的需要保留。
#
# 相机预览单独走一个不带 --required 的 roslaunch（节点 respawn），在主链路节点注册后
# 拉起、收尾时先于主 roslaunch 停掉：相机掉帧超时节点会退出，若并进 fusion.launch，
# --required 会把整条定位链路一起带下来。相机起不来只告警，不影响主链路与退出码。
# ==============================================================================
set -o pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/.." && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="$WS/devel/setup.bash"
ORANGE_SETUP="/home/orangepi/orange/devel/setup.bash"
FUSION_LAUNCH="$DIR/launch/fusion.launch"
CAMERA_LAUNCH="$DIR/launch/camera.launch"
RC_RECORD_DIR="$DIR/rc_record"
LOG_DIR="$WS/logs/fusion"
# 与 start_handheld_test.sh 共用同一把锁文件：新旧入口互斥
LOCK_FILE="/tmp/mid360s_fusion.lock"

export ROS_MASTER_URI="http://192.168.31.32:11311"
export ROS_IP="192.168.31.32"
unset ROS_HOSTNAME 2>/dev/null || true

TTY="/dev/ttyS3"
FCU_URL="${TTY}:921600"
NODES_TIMEOUT=90      # 等全部节点注册（秒）
READY_TIMEOUT=180     # 等首帧 /fastlio/odom_world（秒）
READY_POLL=3          # 等首帧时每轮阻塞上限（秒）：短轮询，好让信号能及时被处理
STOP_TIMEOUT=30       # 收尾时等 roslaunch 退出的上限（秒）

DO_RECORD=1
DO_WINCH=1
DO_CAMERA=1
BAG_PREFIX="noprop"
BAG_DIR="/home/orangepi/bags"

while [ $# -gt 0 ]; do
  case "$1" in
    --no-record) DO_RECORD=0 ;;
    --no-winch) DO_WINCH=0 ;;
    --no-camera) DO_CAMERA=0 ;;
    --bag-prefix) BAG_PREFIX="${2:-}"; shift ;;
    --bag-dir) BAG_DIR="${2:-}"; shift ;;
    -h|--help) sed -n '3,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$BAG_PREFIX" ] || { echo "包名前缀不能为空" >&2; exit 2; }
[ -n "$BAG_DIR" ] || { echo "包目录不能为空" >&2; exit 2; }

mkdir -p "$LOG_DIR" "$BAG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_START_TS="$(date +%s)"        # 收尾时用它筛「本次生成的 bag」
LOGFILE="$LOG_DIR/fusion_${STAMP}.log"
# 本次运行专属目录：放 roslaunch 身份文件和 bag 清单，退出时整个删掉，
# 与上一次运行的文件彻底隔离，不会读到历史残留。
RUN_DIR="$(mktemp -d /tmp/mid360s_fusion.XXXXXX)"
PIDFILE="$RUN_DIR/roslaunch.pid"
CAM_PIDFILE="$RUN_DIR/camera.pid"
CAM_LOGFILE="$LOG_DIR/camera_${STAMP}.log"
trap 'rm -rf "$RUN_DIR"' EXIT

log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
ok()   { printf '[%s] [ OK ] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
warn() { printf '[%s] [WARN] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
err()  { printf '[%s] [FAIL] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE" >&2; }
hr()   { printf '%s\n' "----------------------------------------------------------------" | tee -a "$LOGFILE"; }

EXPECT_NODES="/mavros /livox_lidar_publisher2 /laserMapping /gravity_bridge"
[ "$DO_WINCH" -eq 1 ] && EXPECT_NODES="$EXPECT_NODES /tether_winch_control"

# pgrep -f 会匹配到「命令行里恰好含关键词」的祖先进程（例如通过 ssh/bash -c 调用本
# 脚本时），先算出自身进程树并排除，避免把父 shell 误判成残留节点。
SELF_PIDS=" $$ "
_pp=$$
while [ "$_pp" -gt 1 ]; do
  _parent="$(awk '/^PPid:/{print $2}' "/proc/$_pp/status" 2>/dev/null)"
  [ -z "$_parent" ] && break
  SELF_PIDS="$SELF_PIDS $_parent"
  _pp="$_parent"
done

alive() {
  for _pid in $(pgrep -f "$1" 2>/dev/null); do
    case "$SELF_PIDS" in *" $_pid "*) continue ;; esac
    return 0
  done
  return 1
}

# ------------------------------------------------------------------- 启动锁 ----
# 用 >> 打开（不截断），抢锁失败时还能读到占用者留下的那行；锁随进程退出释放。
exec 200>>"$LOCK_FILE"
if ! flock -n 200; then
  err "已有入口实例在运行（$LOCK_FILE 被占用）"
  _holder="$(tail -n 1 "$LOCK_FILE" 2>/dev/null)"
  [ -n "$_holder" ] && err "占用者：$_holder"
  err "本脚本与 start_handheld_test.sh 互斥，先停掉那个实例再启动"
  exit 1
fi
printf 'fusion.sh pid=%s start=%s\n' "$$" "$STAMP" >&200

SETSID_PID=""         # setsid 包装进程（启动窗口内还没拿到真实 PID 时的兜底身份）
SETSID_ST=""          # 上面那个进程的启动时刻，发信号前核对，防 PID 复用
ROSLAUNCH_PID=""      # 真实 roslaunch PID，由子进程写进 PIDFILE 后回填
LAUNCH_TRIED=0        # 是否已经发起过 roslaunch（收尾时区分「还没起」和「起过又退了」）
CAM_TRIED=0           # 是否已经发起过相机 roslaunch
SHUTTING_DOWN=0
MISSING_NODES=""

# /proc/<pid>/stat 的第 22 个字段 = 进程启动时刻（自开机起的 tick 数）。comm 字段可能
# 含空格和括号，所以先贪婪砍到最后一个 ')' 再数第 20 个字段。exec 不改变 PID 与启动时刻，
# 所以子进程 exec 前后取到的值一致 —— 用它核对身份，防进程早退后 PID 被复用。
proc_starttime() {
  local _s
  _s="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  _s="${_s##*)}"
  # shellcheck disable=SC2086
  set -- $_s
  [ "$#" -ge 20 ] || return 1
  printf '%s\n' "${20}"
}

# 本次 roslaunch 的身份以子进程自己写的 PIDFILE 为准（setsid 有没有 fork 都准），
# 拿到 PID 后核对启动时刻，防止进程早已退出、PID 被别的进程复用时误停无关进程。
# PIDFILE 落地前有个很短的启动窗口，所以这里要能现取 —— 信号如果正好落在这个窗口，
# 收尾路径也拿得到进程，不会把已经起来的 roslaunch 漏在独立会话里。
roslaunch_pid_now() {
  local _pid _st _now
  if [ -n "$ROSLAUNCH_PID" ]; then
    _pid="$ROSLAUNCH_PID"
    _st="$(awk 'NR==1{print $2}' "$PIDFILE" 2>/dev/null)"
  else
    read -r _pid _st <"$PIDFILE" 2>/dev/null || return 1
  fi
  [ -n "$_pid" ] || return 1
  kill -0 "$_pid" 2>/dev/null || return 1
  if [ -n "$_st" ]; then
    _now="$(proc_starttime "$_pid")" || return 1
    [ "$_now" = "$_st" ] || return 1
  fi
  printf '%s\n' "$_pid"
}
roslaunch_alive() { roslaunch_pid_now >/dev/null 2>&1; }

# 相机 roslaunch 的身份同样由包装进程写进 CAM_PIDFILE（PID + 启动时刻），按它核对。
camera_pid_now() {
  local _pid _st _now
  read -r _pid _st <"$CAM_PIDFILE" 2>/dev/null || return 1
  [ -n "$_pid" ] || return 1
  kill -0 "$_pid" 2>/dev/null || return 1
  if [ -n "$_st" ]; then
    _now="$(proc_starttime "$_pid")" || return 1
    [ "$_now" = "$_st" ] || return 1
  fi
  printf '%s\n' "$_pid"
}
camera_alive() { camera_pid_now >/dev/null 2>&1; }

# 相机预览：第二个 roslaunch，不带 --required，同样放进独立会话由本脚本统一收尾。
# 输出只进单独日志，避免 15 Hz 相机日志刷屏主终端。失败只告警，不影响主链路。
start_camera() {
  local _i=0 _cur _miss _n
  log "启动相机预览（launch/camera.launch，日志：$CAM_LOGFILE）"
  rm -f "$CAM_PIDFILE"
  CAM_TRIED=1
  setsid bash -c '
    _pf="$1"; shift; _cmd=("$@")
    _s="$(cat /proc/$$/stat 2>/dev/null)"; _s="${_s##*)}"
    set -- $_s
    printf "%s %s\n" "$$" "${20:-}" >"$_pf"
    exec "${_cmd[@]}"
  ' _ "$CAM_PIDFILE" roslaunch --screen "$CAMERA_LAUNCH" </dev/null >>"$CAM_LOGFILE" 2>&1 &
  while [ ! -s "$CAM_PIDFILE" ] && [ "$_i" -lt 60 ]; do
    sleep 0.2; _i=$((_i + 1))
  done
  if ! camera_alive; then
    warn "相机 roslaunch 没起来，RViz 将没有预览图（看 $CAM_LOGFILE）"
    return 1
  fi
  _i=0
  while [ "$_i" -lt 20 ]; do
    camera_alive || break
    _cur="$(timeout 5 rosnode list 2>/dev/null)"
    _miss=""
    for _n in /ov13855_cam /ov13855_apriltag_vision_node; do
      printf '%s\n' "$_cur" | grep -qx "$_n" || _miss="$_miss $_n"
    done
    if [ -z "$_miss" ]; then
      ok "相机预览已启动：/ov13855_apriltag_vision/image_preview/compressed（有订阅者才生成）"
      return 0
    fi
    sleep 1; _i=$((_i + 1))
  done
  warn "相机节点未全部注册（缺：${_miss:-roslaunch 已退出}），主链路不受影响；看 $CAM_LOGFILE"
  return 1
}

# 先于主 roslaunch 调用：主 roslaunch 带着 ROS Master，先停它会让相机节点注销时卡住。
stop_camera() {
  local _pid _cur _i=0
  [ "$CAM_TRIED" -eq 1 ] || return 0
  while [ ! -s "$CAM_PIDFILE" ] && [ "$_i" -lt 30 ]; do
    sleep 0.1; _i=$((_i + 1))
  done
  _pid="$(camera_pid_now)" || { log "相机 roslaunch 已不在运行"; return 0; }
  log "SIGINT -> 相机 roslaunch $_pid"
  kill -INT "$_pid" 2>/dev/null || true
  _i=0
  while [ "$_i" -lt 30 ]; do
    camera_alive || return 0
    sleep 0.5; _i=$((_i + 1))
    if [ $((_i % 10)) -eq 0 ]; then
      _cur="$(camera_pid_now)" && kill -INT "$_cur" 2>/dev/null
    fi
  done
  warn "相机 roslaunch 未在 15 s 内退出，SIGTERM 兜底"
  _cur="$(camera_pid_now)" && kill -TERM "$_cur" 2>/dev/null
  _i=0
  while [ "$_i" -lt 10 ]; do
    camera_alive || return 0
    sleep 0.5; _i=$((_i + 1))
  done
  warn "相机 roslaunch 仍在运行（pid=$_pid），需手动 kill -INT 处理"
  return 1
}

# ------------------------------------------------------------------- 收尾 ------
# 只停本次启动的 roslaunch（按 PID，绝不按进程名扫描，避免误伤别的实例）。
# 录包节点是 roslaunch 的子进程，收到 roslaunch 转发的 SIGINT 会把 bag 收尾写索引。
stop_roslaunch() {
  local _pid="" _cur="" _i=0

  # 身份先用 PIDFILE 现取：信号正好落在「fork 完、PIDFILE 还没落地」这个窗口时，
  # 回填的 ROSLAUNCH_PID 还是空的，不补这一步就会以为没进程可收而漏掉整条链路。
  if [ "$LAUNCH_TRIED" -eq 1 ]; then
    while [ ! -s "$PIDFILE" ] && [ "$_i" -lt 30 ]; do
      sleep 0.1; _i=$((_i + 1))
    done
  fi
  _pid="$(roslaunch_pid_now)"
  # 还没落地就退一步认 setsid 包装进程（setsid 没 fork 时它与 roslaunch 同 PID），
  # 同样核对启动时刻
  if [ -z "$_pid" ] && [ -n "$SETSID_PID" ] && kill -0 "$SETSID_PID" 2>/dev/null; then
    if [ -z "$SETSID_ST" ] || [ "$(proc_starttime "$SETSID_PID")" = "$SETSID_ST" ]; then
      _pid="$SETSID_PID"
    fi
  fi
  if [ -z "$_pid" ]; then
    _raw="$(awk 'NR==1{print $1}' "$PIDFILE" 2>/dev/null)"
    if [ -n "$_raw" ] && kill -0 "$_raw" 2>/dev/null; then
      warn "pid $_raw 的启动时刻与记录对不上（该 PID 已被别的进程复用），跳过不动它"
    else
      log "没有找到本次启动的 roslaunch（还没起来，或已经退出）"
    fi
    return 0
  fi
  ROSLAUNCH_PID="$_pid"

  log "SIGINT -> roslaunch $_pid（录包节点会先把 bag 收尾）"
  kill -INT "$_pid" 2>/dev/null || true
  _i=0
  while [ "$_i" -lt $((STOP_TIMEOUT * 2)) ]; do
    roslaunch_alive || break
    sleep 0.5; _i=$((_i + 1))
    # 启动瞬间那一下可能落在「子进程 fork 完、还没 exec」的窗口里被丢掉：后台任务的
    # 子进程在 exec 前是忽略 SIGINT 的，信号进不去也收不到。每 5 s 补一次即可 ——
    # 重复 SIGINT 不会打断 roslaunch 的收尾（实测：连发 3 次仍然无残留、bag 索引完好）。
    # 补发前重新核对身份，防止这期间 PID 被复用误伤别的进程。
    if [ $((_i % 10)) -eq 0 ]; then
      _cur="$(roslaunch_pid_now)" && kill -INT "$_cur" 2>/dev/null
    fi
  done
  if roslaunch_alive; then
    warn "roslaunch 未在 ${STOP_TIMEOUT} s 内退出，SIGTERM 兜底"
    _cur="$(roslaunch_pid_now)" && kill -TERM "$_cur" 2>/dev/null
    _i=0
    while [ "$_i" -lt 10 ]; do
      roslaunch_alive || break
      sleep 0.5; _i=$((_i + 1))
    done
  fi
  if roslaunch_alive; then
    warn "roslaunch 仍在运行（pid=$_pid），需手动 kill -INT 处理"
    return 1
  fi
  return 0
}

# 收尾检查：只报告，不再顺手杀别的进程
verify_clean() {
  hr; log "收尾检查"
  local _left=""
  alive "mavros_nod[e]"          && _left="$_left mavros_node"
  alive "livox_ros_driver2_nod[e]" && _left="$_left livox_ros_driver2_node"
  alive "fastlio_mappin[g]"      && _left="$_left fastlio_mapping"
  alive "gravity_bridge_nod[e]"  && _left="$_left gravity_bridge_node"
  alive "bag_recorder_node.p[y]" && _left="$_left bag_recorder_node.py"
  if [ "$DO_WINCH" -eq 1 ] && alive "tether_winch_nod[e]"; then
    _left="$_left tether_winch_node"
  fi
  if [ "$DO_CAMERA" -eq 1 ]; then
    alive "ov13855_cam_nod[e]" && _left="$_left ov13855_cam_node"
    alive "ov13855_apriltag_vision_nod[e]" && _left="$_left ov13855_apriltag_vision_node"
  fi
  if [ -n "$_left" ]; then
    warn "仍有节点进程残留：$_left"
    warn "核对：ps -eo pid,args | grep -E 'mavros_node|livox_ros_driver2_node|fastlio_mapping|gravity_bridge_node|bag_recorder_node|tether_winch_node|ov13855_'"
  else
    ok "链路节点已全部退出"
  fi

  # 录包节点是直接用 rosbag API 写最终 .bag 的，不产生 .bag.active，
  # 所以「没有 .active」根本证明不了收尾成功 —— 得验证本次生成的包索引读不读得出来：
  # 录到一半崩掉的包会留下一个没有索引的 .bag 文件。
  # 路径可能含空格，所以走 find -print0 + read -d ''，不靠词拆分；失败项存数组再遍历。
  local -a _bags=() _bad=()
  local _b
  if ! find "$BAG_DIR" -maxdepth 1 -name "${BAG_PREFIX}_*.bag" -newermt "@$RUN_START_TS" \
         -print0 >"$RUN_DIR/bags.list" 2>/dev/null; then
    warn "扫描 $BAG_DIR 失败，本次 bag 无法校验，请手动检查"
  else
    while IFS= read -r -d '' _b; do _bags+=("$_b"); done <"$RUN_DIR/bags.list"
    if [ "${#_bags[@]}" -eq 0 ]; then
      log "本次没有生成 bag（开关没拨到触发位？）"
    elif ! command -v rosbag >/dev/null 2>&1; then
      warn "找不到 rosbag 命令，跳过 bag 索引校验（本次生成 ${#_bags[@]} 个包）"
      log "最近一个包：${_bags[${#_bags[@]}-1]}"
    else
      for _b in "${_bags[@]}"; do
        rosbag info "$_b" >/dev/null 2>&1 || _bad+=("$_b")
      done
      if [ "${#_bad[@]}" -gt 0 ]; then
        warn "本次有 bag 没正常收尾（索引读不出来，多半是录包中途崩了）："
        for _b in "${_bad[@]}"; do warn "  $_b"; done
      else
        ok "本次生成的 ${#_bags[@]} 个 bag 索引完好"
        log "最近一个包：${_bags[${#_bags[@]}-1]}"
      fi
    fi
  fi

  if ss -ltn 2>/dev/null | grep -q 11311; then
    warn "11311 仍在监听（可能是别的 ROS Master）"
  else
    ok "11311 未监听"
  fi
  if command -v fuser >/dev/null 2>&1; then
    if fuser "$TTY" >/dev/null 2>&1; then
      warn "$TTY 仍被占用"
    else
      ok "$TTY 空闲"
    fi
  fi
}

on_signal() {
  local _sig="$1" _code="$2"
  if [ "$SHUTTING_DOWN" -eq 1 ]; then
    warn "收尾进行中，忽略重复的 SIG$_sig"
    return
  fi
  SHUTTING_DOWN=1
  hr; warn "收到 SIG$_sig：只收拢本次启动的 roslaunch（pid=${ROSLAUNCH_PID:-未启动}）"
  stop_camera
  stop_roslaunch
  verify_clean
  log "退出（日志：$LOGFILE）"
  exit "$_code"
}
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM
# 非交互 shell 把本脚本当后台任务起时，SIGINT 会以「忽略」状态继承下来，POSIX 规定
# 非交互 shell 改不了它，上面的 trap 会静默失效。显式报出来，免得 Ctrl+C 没反应还找不到原因。
case "$(trap -p INT)" in
  *on_signal*) : ;;
  *) warn "SIGINT 被继承成「忽略」（本脚本被当成后台任务启动），Ctrl+C 不生效，请改用 SIGTERM 停止" ;;
esac

# ============================================================== 前置检查 =====
hr; log "融合快速启动（日志：$LOGFILE）"
hr; log "步骤 1/3：前置检查（环境 / 串口 / 防重复启动）"

[ -f "$ROS_SETUP" ] || { err "缺少 $ROS_SETUP"; exit 1; }
[ -f "$WS_SETUP" ]  || { err "缺少 $WS_SETUP，请先在板端 catkin build"; exit 1; }
[ -f "$FUSION_LAUNCH" ] || { err "缺少 $FUSION_LAUNCH"; exit 1; }
if [ "$DO_CAMERA" -eq 1 ]; then
  [ -f "$CAMERA_LAUNCH" ] || { err "缺少 $CAMERA_LAUNCH；或使用 --no-camera"; exit 1; }
fi
[ -f "$RC_RECORD_DIR/start_rc_record.sh" ] || { err "缺少 $RC_RECORD_DIR/start_rc_record.sh"; exit 1; }
[ -f "$RC_RECORD_DIR/package.xml" ] || { err "缺少 $RC_RECORD_DIR/package.xml（roslaunch 靠它定位 rc_record 包）"; exit 1; }
ok "脚本、launch 与 rc_record 包文件齐全（工作空间 $WS）"

set +u; source "$ROS_SETUP"; source "$WS_SETUP"; set -u
if [ "$DO_WINCH" -eq 1 ] || [ "$DO_CAMERA" -eq 1 ]; then
  [ -f "$ORANGE_SETUP" ] || { err "缺少 $ORANGE_SETUP；请构建 orange 或使用 --no-winch --no-camera"; exit 1; }
  set +u; source "$ORANGE_SETUP" --extend; set -u
fi
# rc_record 不是 catkin 包，把 scripts 挂进 ROS_PACKAGE_PATH，roslaunch 才解析得到它
export ROS_PACKAGE_PATH="$DIR${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"

for c in roslaunch rosnode rostopic rosservice rospack flock; do
  command -v "$c" >/dev/null 2>&1 || { err "缺少工具 $c"; exit 1; }
done
for p in mavros livox_ros_driver2 fast_lio fastlio_gravity_bridge rc_record; do
  rospack find "$p" >/dev/null 2>&1 || { err "rospack 找不到包 $p"; exit 1; }
done
if [ "$DO_WINCH" -eq 1 ]; then
  rospack find tether_winch_control >/dev/null 2>&1 || { err "找不到 tether_winch_control，请在 orange 构建"; exit 1; }
  python3 -c 'from tether_winch_control.msg import WinchStatus' || { err "绕线器消息不可用，请重新构建 tether_winch_control"; exit 1; }
fi
if [ "$DO_CAMERA" -eq 1 ]; then
  for p in ov13855_cam ov13855_apriltag_vision; do
    rospack find "$p" >/dev/null 2>&1 || { err "找不到 $p，请在 orange 构建或使用 --no-camera"; exit 1; }
  done
fi
ok "ROS Noetic 与所选功能包齐备"

[ -c "$TTY" ] || { err "飞控串口 $TTY 不存在"; exit 1; }
if [ -w "$TTY" ]; then
  ok "飞控串口 $TTY 存在且可写（$FCU_URL）"
else
  err "$TTY 不可写（检查 dialout 组权限）"; exit 1
fi

stale=""
for pat in "rosmaste[r]" "roslaunc[h]" "mavros_nod[e]" "livox_ros_driver2_nod[e]" \
           "fastlio_mappin[g]" "gravity_bridge_nod[e]" "bag_recorder_node.p[y]"; do
  alive "$pat" && stale="$stale ${pat//[\[\]]/}"
done
if [ "$DO_WINCH" -eq 1 ] && alive "tether_winch_nod[e]"; then
  stale="$stale tether_winch_node"
fi
if [ "$DO_CAMERA" -eq 1 ]; then
  alive "ov13855_cam_nod[e]" && stale="$stale ov13855_cam_node"
  alive "ov13855_apriltag_vision_nod[e]" && stale="$stale ov13855_apriltag_vision_node"
fi
if [ -n "$stale" ]; then
  err "检测到残留进程：$stale"
  err "上一次实例可能没退干净，先收尾（Ctrl+C 或 kill -INT <roslaunch pid>）再启动"
  exit 1
fi
ok "无残留 ROS 进程"

if timeout 5 rosnode list >/dev/null 2>&1; then
  _nodes="$(timeout 5 rosnode list 2>/dev/null)"
  _busy=""
  _check="$EXPECT_NODES /rc_bag_recorder"
  [ "$DO_CAMERA" -eq 1 ] && _check="$_check /ov13855_cam /ov13855_apriltag_vision_node"
  for n in $_check; do
    printf '%s\n' "$_nodes" | grep -qx "$n" && _busy="$_busy $n"
  done
  if [ -n "$_busy" ]; then
    err "ROS Master 上已挂着本链路节点：$_busy"
    err "先停掉旧实例再启动"
    exit 1
  fi
  ok "ROS Master 可达且无本链路节点"
else
  log "当前没有 ROS Master，将由本次 roslaunch 创建"
fi

# ============================================================== 启动 =========
hr; log "步骤 2/3：启动统一 launch（scripts/launch/fusion.launch）"
if [ "$DO_RECORD" -eq 1 ]; then
  log "录包：RC 通道 6 触发，前缀 $BAG_PREFIX，目录 $BAG_DIR"
  REC_ARG=true
else
  warn "已指定 --no-record：不启动录包节点，本次不会有 bag"
  REC_ARG=false
fi

if [ "$DO_WINCH" -eq 1 ]; then
  WINCH_ARG=true
  log "绕线器：开启，使用 orange/src/tether_winch_control/config/winch.yaml"
  warn "绕线器启动即发送备用 DUTY；定位通信就绪后按 YAML 自动启用；退出会尝试发送备用 DUTY"
else
  WINCH_ARG=false
  log "已指定 --no-winch：不启动绕线器控制，不由 fusion 发送 DUTY"
fi
if [ "$DO_CAMERA" -eq 1 ]; then
  log "相机预览：主链路节点注册后单独拉起（非 required，掉线不影响定位）"
else
  log "已指定 --no-camera：不启动相机预览"
fi

# roslaunch 放进独立会话（setsid）：终端信号不会绕过本脚本直接打到它，由 on_signal
# 统一发一次 SIGINT，避免重复信号打断 roslaunch 的收尾流程。真实 PID 由子进程自己
# 写进 PIDFILE，不依赖 setsid 有没有 fork。
# tee 忽略 SIGINT：否则 Ctrl+C 会先杀掉 tee，roslaunch 再写日志就吃 SIGPIPE。
rm -f "$PIDFILE"
LAUNCH_TRIED=1   # 先置位再 fork：这段窗口里收到信号，收尾也知道「已经发起过」
# 包装进程先把「自己的 PID + 启动时刻」写进 PIDFILE 再 exec：exec 不改这两项，
# 所以收尾时读到的身份依然有效，可用来核对、防 PID 复用。
setsid --wait bash -c '
  _pf="$1"; shift; _cmd=("$@")
  _s="$(cat /proc/$$/stat 2>/dev/null)"; _s="${_s##*)}"
  set -- $_s
  printf "%s %s\n" "$$" "${20:-}" >"$_pf"
  exec "${_cmd[@]}"
' _ "$PIDFILE" \
  roslaunch --screen --required "$FUSION_LAUNCH" \
    fcu_url:="$FCU_URL" fcu_protocol:=v1.0 tgt_system:=1 tgt_component:=1 \
    send_to_mavros:=true "record:=$REC_ARG" "winch:=$WINCH_ARG" bag_prefix:="$BAG_PREFIX" bag_dir:="$BAG_DIR" \
  </dev/null > >( trap '' INT; exec tee -a "$LOGFILE" ) 2>&1 &
SETSID_PID=$!
SETSID_ST="$(proc_starttime "$SETSID_PID" 2>/dev/null)"

for _ in $(seq 1 60); do
  [ -s "$PIDFILE" ] && break
  sleep 0.2
done
ROSLAUNCH_PID="$(awk 'NR==1{print $1}' "$PIDFILE" 2>/dev/null)"
if [ -z "$ROSLAUNCH_PID" ]; then
  err "roslaunch 没能在 12 s 内起来（PID 文件为空：$PIDFILE）"
  err "看日志：$LOGFILE"
  wait "$SETSID_PID" 2>/dev/null
  exit 1
fi
log "roslaunch pid=$ROSLAUNCH_PID（节点日志同时进终端和 $LOGFILE）"

# ============================================================== 就绪 =========
hr; log "步骤 3/3：等待节点注册与桥接输出"
T0=$SECONDS
wait_for_nodes() {
  local _t=0 _cur _miss _n
  while [ "$_t" -lt "$NODES_TIMEOUT" ]; do
    roslaunch_alive || return 2
    _cur="$(timeout 5 rosnode list 2>/dev/null)"
    _miss=""
    for _n in $EXPECT_NODES; do
      printf '%s\n' "$_cur" | grep -qx "$_n" || _miss="$_miss $_n"
    done
    [ -z "$_miss" ] && return 0
    MISSING_NODES="$_miss"
    sleep 2; _t=$((_t + 2))
  done
  return 1
}
wait_for_nodes; NODES_RC=$?

if [ "$NODES_RC" -eq 2 ]; then
  err "roslaunch 在节点注册完成前就退出了 —— 启动失败，看上面的日志"
elif [ "$NODES_RC" -eq 0 ]; then
  ok "节点已启动（耗时 $((SECONDS - T0)) s）：$EXPECT_NODES"
  if [ "$DO_RECORD" -eq 1 ]; then
    if timeout 5 rosnode list 2>/dev/null | grep -qx /rc_bag_recorder; then
      ok "录包节点已注册 /rc_bag_recorder（通道 6 拨到触发位开始录）"
    else
      warn "录包节点没注册上，检查上面的日志"
    fi
  fi
  [ "$DO_CAMERA" -eq 1 ] && start_camera
  hr; log "等待桥接输出里程计：首帧 /fastlio/odom_world（桥接收到有效 gravity_init 才发）"
  # 每轮只阻塞 READY_POLL 秒：Bash 要等前台命令结束才处理 trap，
  # 一次 timeout 180 的同步等待会让 SIGTERM 拖到 180 s，期间节点崩了也报不出来。
  _t=0; _ready=0
  while [ "$_t" -lt "$READY_TIMEOUT" ]; do
    if timeout "$READY_POLL" rostopic echo -n1 /fastlio/odom_world >/dev/null 2>&1; then
      _ready=1; break
    fi
    roslaunch_alive || break
    _t=$((_t + READY_POLL))
  done
  if [ "$_ready" -eq 1 ]; then
    ok "桥接已输出里程计（从启动到首帧共 $((SECONDS - T0)) s）"
    log "注意：这只说明 FAST-LIO→桥接这一段通了。MAVROS 有没有连上飞控、PX4 有没有"
    log "      接受外部定位，本入口不检查（要验收请用 start_handheld_test.sh）"
  elif ! roslaunch_alive; then
    warn "等首帧期间 roslaunch 已经退出（见上面的日志）"
  else
    warn "${READY_TIMEOUT} s 内没等到 /fastlio/odom_world 首帧，链路仍在运行"
    warn "多数是初始化时机体没静止（重力初始化不过），或雷达无数据"
    warn "初始化保持静止即可，无须水平（桥接默认固定安装补偿 20 度）"
    warn "仅当显式改用旧的 mounting_source:=gravity 才要求初始化时机体水平"
  fi
else
  warn "${NODES_TIMEOUT} s 内未等到全部节点注册，缺：${MISSING_NODES:-未知}（链路仍在运行，见上面日志）"
fi

# ============================================================== 前台常驻 =====
if roslaunch_alive; then
  echo
  echo "=================================================================="
  echo "  链路运行中 —— 停止：Ctrl+C"
  echo "=================================================================="
  echo "  包：$BAG_DIR/${BAG_PREFIX}_000001.bag（每拨一次新增一个序号）"
  echo "  日志：$LOGFILE"
  echo "  roslaunch pid=$ROSLAUNCH_PID（独立会话，收尾只能靠本脚本）"
  echo
  if [ "$DO_RECORD" -eq 1 ]; then
    cat <<'TIP'
遥控器：
  1) 通道 6 拨到「触发位」（PWM 大的那一侧）开始录 bag，拨回去 1 s 后收尾。
  2) 现场确认极性：rostopic echo /mavros/rc/in 看 channels[5]，
     或另开终端 rostopic echo /rc_record/status 看 state=REC/IDLE。
  3) 飞控 ulog 同向（SDLOG_MODE=3 + RC_MAP_AUX1=6）时，同一个开关同时录 bag 与 ulog。
  4) 一次测试内不要重启链路：world 原点绑定 FAST-LIO 首次初始化，重启会跳变。
TIP
    echo
  fi
  HB=0
  while roslaunch_alive; do
    sleep 2
    HB=$((HB + 2))
    if [ "$HB" -ge 30 ]; then
      HB=0
      _armed="$(timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | sed -n 's/^armed: //p' | head -1)"
      _rec="$(timeout 5 rostopic echo -n1 /rc_record/status 2>/dev/null | sed -n 's/^data: "\(state=[^ ]*\).*/\1/p')"
      _cam=""
      if [ "$DO_CAMERA" -eq 1 ]; then
        if camera_alive; then _cam=" · cam=up"; else _cam=" · cam=down"; fi
      fi
      log "运行中 · armed=${_armed:-?} · ${_rec:-rec=n/a}${_cam}"
    fi
  done
fi

# ============================================================== 退出码 =======
if [ -n "$SETSID_PID" ]; then
  wait "$SETSID_PID"; RL_RC=$?
else
  RL_RC=0
fi
hr
# 能走到这里说明没收到过停止信号（收到信号的都在 on_signal 里 exit 掉了），
# 所以 roslaunch 是自己退的：--required 之下就是某个节点先退出把整条链路带下来了。
# roslaunch 遇到 required 节点退出也给退出码 0，这里补一个非 0，
# 免得 systemd / 上层脚本把「节点崩了」当成「正常结束」。
if [ "$RL_RC" -ne 0 ]; then
  err "roslaunch 异常退出（退出码 $RL_RC），往上翻日志里的第一个报错"
  EXIT_RC="$RL_RC"
elif [ "$SHUTTING_DOWN" -eq 0 ]; then
  err "roslaunch 在没收到停止信号的情况下自行退出"
  err "多半是某个节点先退出触发了 required 全体收拢，找日志里的 \"REQUIRED process [...] has died!\" 确认是哪个"
  EXIT_RC=1
else
  ok "roslaunch 已退出（退出码 0）"
  EXIT_RC=0
fi
stop_camera
verify_clean
log "退出（日志：$LOGFILE）"
exit "$EXIT_RC"
