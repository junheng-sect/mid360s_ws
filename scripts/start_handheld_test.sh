#!/usr/bin/env bash
# ==============================================================================
# 台架 / 手持 / 无桨测试 一键启动
#   MAVROS(飞控串口) + MID-360S 驱动 + FAST-LIO2 + 重力对齐桥接(位姿送飞控)
#   + RC 开关触发录包节点（遥控器通道 6 拨到触发位即开始录 bag）
#
# 路径: /home/orangepi/mid360s_ws/scripts/start_handheld_test.sh
# 用法: ./start_handheld_test.sh [选项]
#   --check             只做前置检查，不启动任何节点
#   --no-lidar          调试：只起 MAVROS + 录包节点，用于单独验证「开关 -> 录包」
#   --no-record         不启动 RC 触发录包节点
#   --bag-prefix NAME   包名前缀（默认 noprop，产出 NAME_000001.bag）
#   --bag-dir PATH      包输出目录（默认 /home/orangepi/bags）
#   --fix-params        飞控参数不符时自动写入（含 RC 录包所需的 SDLOG_MODE=3）
#   --require-disarmed  飞控已解锁则中止（默认只告警，适配无桨台架测试）
#   --no-net-check      跳过雷达网口/ping 检查（调试用）
#   -h|--help           帮助
#
# 本脚本是「逐项体检」入口；只想快速把链路拉起来请用同目录 fusion.sh（一条统一
# launch 起全部节点，不 ping、不核对飞控参数、不逐话题测频）。两个入口共用同一把
# 锁，互相排斥。
#
# 前台运行：Ctrl+C / SIGTERM 停止本次启动的节点；录包节点先收 SIGINT，bag 正常写索引。
# 本脚本不修改 mid360s_ws 源码，不写飞控参数（除显式 --fix-params）。
# ==============================================================================
set -o pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/.." && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="$WS/devel/setup.bash"
FASTLIO_LAUNCHER="$DIR/start_mid360s_fastlio.sh"
RC_RECORD_DIR="$DIR/rc_record"
RC_RECORD_LAUNCHER="$RC_RECORD_DIR/start_rc_record.sh"
LOG_DIR="$WS/logs/handheld"
# 与 fusion.sh 共用同一把锁文件：新旧入口互斥
LOCK_FILE="/tmp/mid360s_fusion.lock"

export ROS_MASTER_URI="http://192.168.31.32:11311"
export ROS_IP="192.168.31.32"
unset ROS_HOSTNAME 2>/dev/null || true

WIFI_IP="192.168.31.32"
LIDAR_IFACE_IP="192.168.1.5"
LIDAR_IP="192.168.1.194"
TTY="/dev/ttyS3"
FCU_URL="${TTY}:921600"

CHECK_ONLY=0; FIX_PARAMS=0; NET_CHECK=1
DO_RECORD=1; DO_LIDAR=1; REQUIRE_DISARMED=0
BAG_PREFIX="noprop"; BAG_DIR="/home/orangepi/bags"

while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    --no-lidar) DO_LIDAR=0; NET_CHECK=0 ;;
    --no-record) DO_RECORD=0 ;;
    --bag-prefix) BAG_PREFIX="${2:-}"; shift ;;
    --bag-dir) BAG_DIR="${2:-}"; shift ;;
    --fix-params) FIX_PARAMS=1 ;;
    --require-disarmed) REQUIRE_DISARMED=1 ;;
    --no-net-check) NET_CHECK=0 ;;
    -h|--help) sed -n '2,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$BAG_PREFIX" ] || { echo "包名前缀不能为空" >&2; exit 2; }

mkdir -p "$LOG_DIR" "$BAG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_START_TS="$(date +%s)"        # 收尾时用它筛「本次生成的 bag」
LOGFILE="$LOG_DIR/handheld_${STAMP}.log"
# 本次运行专属的 PID 目录：mktemp -d 保证唯一，上一次异常退出留下的 pid 文件绝不会被
# 这一次读到，也就不可能拿历史 PID 去误停无关进程；收尾时整个目录一起删掉。
RUN_DIR="$(mktemp -d /tmp/mid360s_handheld.XXXXXX)"
PIDFILE_MAVROS="$RUN_DIR/mavros.pid"
PIDFILE_FASTLIO="$RUN_DIR/fastlio.pid"
PIDFILE_REC="$RUN_DIR/rec.pid"
trap 'rm -rf "$RUN_DIR"' EXIT

log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
ok()   { printf '[%s] [ OK ] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
warn() { printf '[%s] [WARN] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
err()  { printf '[%s] [FAIL] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE" >&2; }
hr()   { printf '%s\n' "----------------------------------------------------------------" | tee -a "$LOGFILE"; }

NODE_PAT="mavros_nod[e]|livox_ros_driver2_nod[e]|fastlio_mappin[g]|gravity_bridge_nod[e]|bag_recorder_node.p[y]"

# 本次启动的子进程：收尾只认在 STARTED_PIDFILES 里登记过的组件，不按进程名扫描，
# 也不认任何没登记过的 pid 文件。
SHUTTING_DOWN=0
STARTED_PIDFILES=" "

pid_alive() { [ -n "$1" ] && kill -0 "$1" 2>/dev/null; }

# /proc/<pid>/stat 的第 22 个字段 = 进程启动时刻（自开机起的 tick 数）。
# comm 字段可能含空格和括号，所以先贪婪砍到最后一个 ')' 再数第 20 个字段。
# exec 不改变 PID 和启动时刻，所以子进程 exec 前后取到的值一致 —— 用它核对身份，防 PID 复用。
proc_starttime() {
  local _s
  _s="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  _s="${_s##*)}"
  # shellcheck disable=SC2086
  set -- $_s
  [ "$#" -ge 20 ] || return 1
  printf '%s\n' "${20}"
}

child_started() { # 本次是否启动过这个组件
  case "$STARTED_PIDFILES" in *" $1 "*) return 0 ;; *) return 1 ;; esac
}

# 取本次启动的子进程 PID。只认登记过的组件；拿到 PID 后核对启动时刻，防止子进程早已
# 退出、PID 被别的进程复用时误停无关进程。核不上就什么都不输出（返回非 0）。
child_pid() { # child_pid <pidfile>
  child_started "$1" || return 1
  local _pid _st _now
  read -r _pid _st <"$1" 2>/dev/null || return 1
  [ -n "$_pid" ] || return 1
  pid_alive "$_pid" || return 1
  if [ -n "$_st" ]; then
    _now="$(proc_starttime "$_pid")" || return 1
    [ "$_now" = "$_st" ] || return 1
  fi
  printf '%s\n' "$_pid"
}

# 停一个组件。身份每次现取现核对（不看缓存），核不上就不动它。
stop_child() { # stop_child <pidfile> <名字>
  local _pf="$1" _name="$2" _pid _raw _i=0
  child_started "$_pf" || return 0                 # 本次没启动过：绝不碰
  # 启动窗口内 pidfile 可能还没落地（子进程 exec 前就会写），短等一下再取
  while [ ! -s "$_pf" ] && [ "$_i" -lt 10 ]; do sleep 0.1; _i=$((_i + 1)); done
  if _pid="$(child_pid "$_pf")"; then
    log "停止$_name（pid=$_pid）"; kill -INT "$_pid" 2>/dev/null || true
  else
    # 有记录、进程也还在，但启动时刻对不上 => PID 被复用，明确报出来而不是默默放过
    read -r _raw _ <"$_pf" 2>/dev/null || true
    if [ -n "$_raw" ] && pid_alive "$_raw"; then
      warn "pid $_raw 的启动时刻与记录对不上（该 PID 已被别的进程复用），跳过不动它"
    fi
    return 0
  fi
}

# 等一组组件退出（每轮 0.5 s，共 <轮数> 轮）。每轮都重新核对身份，所以中途 PID 被复用
# 也不会误伤别的进程。还在的每隔 5 s（10 轮）补一次 SIGINT：启动瞬间那一下可能落在
# 「子进程 fork 完、还没 exec」的窗口里被丢掉 —— 后台任务的子进程在 exec 前忽略 SIGINT，
# 信号进不去。重复 SIGINT 不会打断 roslaunch 的收尾（实测连发 3 次仍然无残留、bag 索引完好）。
wait_gone() { # wait_gone <轮数> <pidfile> [<pidfile> ...]
  local _rounds="$1"; shift
  local -a _pfs=("$@")
  local _i=0 _pf _p _alive
  while [ "$_i" -lt "$_rounds" ]; do
    _alive=0
    for _pf in "${_pfs[@]}"; do
      _p="$(child_pid "$_pf" 2>/dev/null)" || continue
      if pid_alive "$_p"; then
        _alive=1
        [ $((_i % 10)) -eq 9 ] && kill -INT "$_p" 2>/dev/null
      fi
    done
    [ "$_alive" -eq 0 ] && return 0
    sleep 0.5; _i=$((_i + 1))
  done
  return 1
}

# 起一个独立会话的子进程，并把「PID + 启动时刻」写进 pidfile 回填 CHILD_PID。
# 独立会话让终端的 Ctrl+C 绕不到子进程，信号统一由 on_signal 转发，避免重复 SIGINT
# 打断 roslaunch 的收尾；身份由子进程自己写文件，不依赖 setsid 是否会 fork，
# 记下启动时刻是为了收尾时核对身份、防 PID 复用（exec 不改这两项）。
start_child() { # start_child <pidfile> <logfile> <命令...>
  local _pf="$1" _lf="$2"; shift 2
  rm -f "$_pf"
  STARTED_PIDFILES="$STARTED_PIDFILES$_pf "   # 先登记再 fork：这段窗口里收尾也找得到身份
  setsid --wait bash -c '
    _pf="$1"; shift; _cmd=("$@")
    _s="$(cat /proc/$$/stat 2>/dev/null)"; _s="${_s##*)}"
    set -- $_s
    printf "%s %s\n" "$$" "${20:-}" >"$_pf"
    exec "${_cmd[@]}"
  ' _ "$_pf" "$@" </dev/null >"$_lf" 2>&1 &
  local _i=0
  while [ "$_i" -lt 50 ]; do [ -s "$_pf" ] && break; sleep 0.1; _i=$((_i + 1)); done
  CHILD_PID="$(awk 'NR==1{print $1}' "$_pf" 2>/dev/null)"
}

# ---------------------------------------------------------------- 退出清理 ----
stop_everything() {
  hr; log "开始收尾（只停本次启动的进程）"
  # 录包节点先停：SIGINT 让它把 bag 正常收尾（写索引）
  stop_child "$PIDFILE_REC" "录包节点"
  wait_gone 20 "$PIDFILE_REC"
  stop_child "$PIDFILE_FASTLIO" "雷达/FAST-LIO/桥接"
  stop_child "$PIDFILE_MAVROS" "MAVROS"
  wait_gone 50 "$PIDFILE_FASTLIO" "$PIDFILE_MAVROS"
  if alive "$NODE_PAT"; then
    warn "仍有节点进程残留，请用 ps 检查后手动清理"
  else
    ok "所有节点已停止，11311 与 $TTY 已释放"
  fi
  # 录包节点直接用 rosbag API 写最终 .bag，不产生 .bag.active，
  # 所以「没有 .active」证明不了收尾成功 —— 得验证本次生成的包索引读不读得出来。
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
}
on_signal() {
  local _code="${1:-130}"
  if [ "$SHUTTING_DOWN" -eq 1 ]; then warn "收尾进行中，忽略重复信号"; return; fi
  SHUTTING_DOWN=1
  log "收到中断信号"
  stop_everything
  log "退出（日志：$LOGFILE）"
  exit "$_code"
}

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
count_alive() {
  _n=0
  for _pid in $(pgrep -f "$1" 2>/dev/null); do
    case "$SELF_PIDS" in *" $_pid "*) continue ;; esac
    _n=$((_n+1))
  done
  echo "$_n"
}

# 任何阶段被 Ctrl+C / SIGTERM 中断都走同一套收尾（早设，避免中途退出留下孤儿节点）
trap 'on_signal 130' INT
trap 'on_signal 143' TERM
# 非交互 shell 把本脚本当后台任务起时，SIGINT 会以「忽略」状态继承下来，POSIX 规定
# 非交互 shell 改不了它，上面的 trap 会静默失效。显式报出来，免得 Ctrl+C 没反应还找不到原因。
case "$(trap -p INT)" in
  *on_signal*) : ;;
  *) warn "SIGINT 被继承成「忽略」（本脚本被当成后台任务启动），Ctrl+C 不生效，请改用 SIGTERM 停止" ;;
esac

# ------------------------------------------------------------ 小工具 ----------
state_connected() { timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q "connected: True"; }
pget()            { timeout 20 rosservice call /mavros/param/get "{param_id: '$1'}" 2>/dev/null | awk '/integer:/{print $2; exit}'; }
pset_int()        { timeout 20 rosservice call /mavros/param/set "{param_id: '$1', value: {integer: $2, real: 0.0}}" >/dev/null 2>&1; }
topic_hz_ok()     { [ "$(timeout 8 rostopic hz "$1" 2>&1 | grep -c 'average rate')" -gt 0 ]; }

wait_for() { # wait_for <描述> <秒数> <命令...>
  local desc="$1" secs="$2"; shift 2
  local t=0
  while [ "$t" -lt "$secs" ]; do
    if "$@" >/dev/null 2>&1; then ok "$desc"; return 0; fi
    sleep 2; t=$((t+2))
  done
  return 1
}

# ============================================================== 前置检查 =====
hr; log "台架/无桨测试 一键启动（日志：$LOGFILE）"
hr; log "步骤 1/5：前置检查"

[ -f "$ROS_SETUP" ] || { err "缺少 $ROS_SETUP"; exit 1; }
[ -f "$WS_SETUP" ]  || { err "缺少 $WS_SETUP，请先在板端 catkin build"; exit 1; }
[ -f "$FASTLIO_LAUNCHER" ] || { err "缺少 $FASTLIO_LAUNCHER"; exit 1; }
[ -f "$RC_RECORD_LAUNCHER" ] || { err "缺少 $RC_RECORD_LAUNCHER"; exit 1; }
ok "脚本与工作空间路径齐全（工作空间 $WS）"

set +u; source "$ROS_SETUP"; source "$WS_SETUP"; set -u
for c in roslaunch rosnode rostopic rosservice rosparam rospack flock ping ip; do
  command -v "$c" >/dev/null 2>&1 || { err "缺少工具 $c"; exit 1; }
done
for p in livox_ros_driver2 fast_lio fastlio_gravity_bridge mavros; do
  rospack find "$p" >/dev/null 2>&1 || { err "rospack 找不到包 $p"; exit 1; }
done
ok "ROS Noetic + 4 个必需包齐备"

stale=""
for pat in "rosmaste[r]" "roslaunc[h]" "mavros_nod[e]" "livox_ros_driver2_nod[e]" "fastlio_mappin[g]" "gravity_bridge_nod[e]" "bag_recorder_node.p[y]" "rosbag recor[d]"; do
  alive "$pat" && stale="$stale $pat"
done
if [ -n "$stale" ]; then
  err "检测到残留进程：$stale"
  err "请先收尾上一个实例（Ctrl+C 或 kill -INT <roslaunch pid>）后再启动"
  exit 1
fi
ok "无残留 ROS 进程"

# 抢到就一直持有（fd 200 随进程退出才释放），否则新旧入口会同时跑起来
exec 200>>"$LOCK_FILE"
if flock -n 200; then
  ok "启动锁已持有（$LOCK_FILE）"
  printf 'start_handheld_test.sh pid=%s start=%s\n' "$$" "$STAMP" >&200
else
  _holder="$(tail -n 1 "$LOCK_FILE" 2>/dev/null)"
  err "另一个入口实例在运行（$LOCK_FILE 被占用）${_holder:+：$_holder}"
  err "本脚本与 fusion.sh 互斥，先停掉那个实例再启动"
  exit 1
fi

if [ "$NET_CHECK" -eq 1 ]; then
  ip -4 addr show 2>/dev/null | grep -qw "$WIFI_IP" || { err "Wi-Fi 未配置 $WIFI_IP"; exit 1; }
  ok "板端 Wi-Fi IP $WIFI_IP"
  if ! ip -4 addr show 2>/dev/null | grep -qw "$LIDAR_IFACE_IP"; then
    err "雷达网口未配置 $LIDAR_IFACE_IP（enP3p49s0 无载波：雷达没上电或网线没插）"
    err "排查：ip -4 -br addr show enP3p49s0  |  nmcli device status"
    err "只想起 MAVROS + 录包节点调试，可加 --no-lidar"
    exit 1
  fi
  ok "雷达网口 IP $LIDAR_IFACE_IP"
  if ping -c 1 -W 2 "$LIDAR_IP" >/dev/null 2>&1; then
    ok "雷达 $LIDAR_IP 可达"
  else
    err "雷达 $LIDAR_IP ping 不通（雷达未上电 / 网线 / 网口）"; exit 1
  fi
else
  warn "已按要求跳过雷达网络检查"
fi

[ -c "$TTY" ] || { err "飞控串口 $TTY 不存在"; exit 1; }
if [ -w "$TTY" ]; then ok "飞控串口 $TTY 存在且可写"; else err "$TTY 不可写（检查 dialout 组权限）"; exit 1; fi
[ "$DO_LIDAR" -eq 1 ] || warn "已指定 --no-lidar：只起 MAVROS + 录包节点，不启动雷达/FAST-LIO"

if [ "$CHECK_ONLY" -eq 1 ]; then
  hr; log "飞控参数检查需要启动 MAVROS，--check 模式不做"
  log "前置检查全部通过，可直接运行：$0"
  exit 0
fi

# ============================================================== 启动 MAVROS ==
hr; log "步骤 2/5：启动 MAVROS（$FCU_URL）"
start_child "$PIDFILE_MAVROS" "$LOG_DIR/mavros_${STAMP}.log" \
  roslaunch mavros px4.launch \
  fcu_url:="$FCU_URL" fcu_protocol:=v1.0 tgt_system:=1 tgt_component:=1 gcs_url:=""
log "MAVROS pid=$CHILD_PID（日志：$LOG_DIR/mavros_${STAMP}.log）"
wait_for "MAVROS 已连接飞控" 60 state_connected || {
  err "60 s 内未连上飞控，看日志：$LOG_DIR/mavros_${STAMP}.log"
  err "常见原因：飞控未上电 / 串口被占用 / 波特率不符"
  stop_everything; exit 1
}

armed="$(timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | awk '/^armed:/{print $2}')"
mode="$(timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | awk '/^mode:/{print $2}')"
log "飞控状态：armed=$armed mode=$mode"

# ========================================================== 飞控参数校验 =====
hr; log "步骤 3/5：飞控参数校验（视觉位姿融合 + RC 录包配对）"
timeout 60 rosservice call /mavros/param/pull "{force_pull: true}" >/dev/null 2>&1
EV_CTRL="$(pget EKF2_EV_CTRL)"; GPS_CTRL="$(pget EKF2_GPS_CTRL)"
HGT_REF="$(pget EKF2_HGT_REF)"; SDLOG_MODE="$(pget SDLOG_MODE)"
AUX1_MAP="$(pget RC_MAP_AUX1)"
log "EKF2_EV_CTRL=$EV_CTRL  EKF2_GPS_CTRL=$GPS_CTRL  EKF2_HGT_REF=$HGT_REF  SDLOG_MODE=$SDLOG_MODE  RC_MAP_AUX1=$AUX1_MAP"

need_fix=0
case "${EV_CTRL:-x}" in ''|*[!0-9]*) err "读不到 EKF2_EV_CTRL"; stop_everything; exit 1 ;; esac
if [ $((EV_CTRL & 1)) -eq 0 ]; then
  err "EKF2_EV_CTRL=$EV_CTRL 未开启「水平位置融合」(bit0)：飞控不会使用激光位姿的水平定位"
  need_fix=1
else
  ok "EV 水平位置融合已开启（bit0）"
fi
if [ $((EV_CTRL & 2)) -ne 0 ]; then ok "EV 垂直位置融合已开启（bit1）"; else warn "EV 垂直位置融合未开（bit1）：高度不跟随视觉"; fi
if [ $((EV_CTRL & 8)) -ne 0 ]; then ok "EV 偏航融合已开启（bit3）"; else warn "EV 偏航融合未开（bit3）：航向将依赖磁罗盘"; fi
[ $((EV_CTRL & 4)) -ne 0 ] && warn "EV 3D 速度位(bit2)已开，但本链路只发位姿不发速度，该位无数据源"
if [ "${GPS_CTRL:-x}" = "0" ]; then ok "GPS 融合已关闭"; else warn "EKF2_GPS_CTRL=$GPS_CTRL 非 0，无 GPS 时建议置 0"; need_fix=1; fi
if [ "${HGT_REF:-x}" = "3" ]; then ok "高度主参考为 Vision(3)"; else warn "EKF2_HGT_REF=$HGT_REF 非 Vision(3)"; need_fix=1; fi

if [ "$DO_RECORD" -eq 1 ]; then
  if [ "${SDLOG_MODE:-x}" = "3" ]; then
    ok "SDLOG_MODE=3（AUX1 拨到记录位才记 ulog，与 bag 同一开关）"
  else
    warn "SDLOG_MODE=$SDLOG_MODE 不是 3：拨开关只会录 bag，飞控不会产生 ulog"
    warn "改法（reboot_required，需重启飞控）：rosservice call /mavros/param/set \"{param_id: 'SDLOG_MODE', value: {integer: 3, real: 0.0}}\""
    need_fix=1
  fi
  if [ "${AUX1_MAP:-x}" = "6" ]; then
    ok "RC_MAP_AUX1=6（AUX1 就是通道 6，与录包节点同一开关）"
  else
    warn "RC_MAP_AUX1=$AUX1_MAP 不是 6：AUX1 与录包节点看的通道不是同一个开关"
    need_fix=1
  fi
fi

if [ "$need_fix" -eq 1 ]; then
  if [ "$FIX_PARAMS" -eq 1 ]; then
    warn "按 --fix-params 写入：EKF2_EV_CTRL=11、EKF2_GPS_CTRL=0、EKF2_HGT_REF=3"
    pset_int EKF2_EV_CTRL 11; pset_int EKF2_GPS_CTRL 0; pset_int EKF2_HGT_REF 3
    if [ "$DO_RECORD" -eq 1 ]; then
      warn "同时写入 SDLOG_MODE=3、RC_MAP_AUX1=6（SDLOG_MODE 需重启飞控才生效）"
      pset_int SDLOG_MODE 3; pset_int RC_MAP_AUX1 6
    fi
    sleep 1
    log "回读：EKF2_EV_CTRL=$(pget EKF2_EV_CTRL) EKF2_GPS_CTRL=$(pget EKF2_GPS_CTRL) EKF2_HGT_REF=$(pget EKF2_HGT_REF) SDLOG_MODE=$(pget SDLOG_MODE) RC_MAP_AUX1=$(pget RC_MAP_AUX1)"
  else
    err "参数与本次测试口径不符；确认后加 --fix-params 重跑，或在 QGC 里改完再启动"
    stop_everything; exit 3
  fi
fi

if [ "$armed" = "True" ]; then
  warn "飞控当前处于 armed 状态：请确认已卸桨、周围无人，再继续"
  if [ "$REQUIRE_DISARMED" -eq 1 ]; then
    err "指定了 --require-disarmed，已解锁 => 中止"
    stop_everything; exit 1
  fi
fi

# ================================================ 启动雷达 + FAST-LIO + 桥接 =
if [ "$DO_LIDAR" -eq 1 ]; then
  hr; log "步骤 4/5：启动 MID-360S 驱动 + FAST-LIO2 + 重力对齐桥接（位姿送 /mavros/vision_pose/pose_cov）"
  cd "$WS"
  start_child "$PIDFILE_FASTLIO" "$LOG_DIR/fastlio_${STAMP}.log" "$FASTLIO_LAUNCHER" --send-to-mavros
  log "雷达/FAST-LIO/桥接 pid=$CHILD_PID（日志：$LOG_DIR/fastlio_${STAMP}.log）"

  wait_for "重力桥接节点已注册" 60 bash -c 'rosnode list 2>/dev/null | grep -qx /gravity_bridge' || {
    err "节点未起来，看日志：$LOG_DIR/fastlio_${STAMP}.log"; stop_everything; exit 1
  }
  wait_for "/livox/lidar 有数据" 45 topic_hz_ok /livox/lidar  || { err "雷达点云无输出"; stop_everything; exit 1; }
  wait_for "/livox/imu 有数据" 30 topic_hz_ok /livox/imu     || { err "雷达 IMU 无输出"; stop_everything; exit 1; }
  wait_for "FAST-LIO 输出 /Odometry" 45 topic_hz_ok /Odometry || { err "FAST-LIO 无输出"; stop_everything; exit 1; }

  hr; log "步骤 5/5：就绪自检"
  wait_for "桥接输出 /fastlio/vision_pose_cov" 60 topic_hz_ok /fastlio/vision_pose_cov || {
    err "桥接未输出（多数是没收到 /fastlio/gravity_init：初始化时机体必须保持静止）"
    err "初始化静止即可、无须水平（默认固定安装补偿 mounting_source:=fixed, 20 度）；"
    err "只有显式改用旧的 mounting_source:=gravity 时才要求初始化时机体水平"
    err "看日志：$LOG_DIR/fastlio_${STAMP}.log"; stop_everything; exit 1
  }
  info="$(timeout 5 rostopic info /mavros/vision_pose/pose_cov 2>&1)"
  echo "$info" | grep -q "/gravity_bridge" && ok "发布者 /gravity_bridge" || { err "发布者异常：$info"; stop_everything; exit 1; }
  echo "$info" | grep -q "/mavros" && ok "订阅者 /mavros（飞控已在收位姿）" || { err "MAVROS 未订阅 /mavros/vision_pose/pose_cov"; stop_everything; exit 1; }

  wait_for "EKF 已获得本地水平位置（pos_horiz_rel=True）" 40 \
    bash -c '[ "$(timeout 8 rostopic echo -n1 /mavros/estimator_status 2>/dev/null | awk "/^pos_horiz_rel_status_flag:/{print \$2}")" = "True" ]' \
    || warn "EKF 仍未报告有效水平位置，检查 EKF2_EV_CTRL/GPS_CTRL 与飞控日志"

  for x in /fastlio/vision_pose_cov /mavros/local_position/pose; do
    log "频率 $x: $(timeout 7 rostopic hz "$x" 2>&1 | awk '/average rate/{print $3 " Hz"}' | head -1)"
  done
else
  hr; log "步骤 4/5、5/5：已跳过雷达链路（--no-lidar），只做录包链路"
fi
ok "节点清单：$(rosnode list 2>/dev/null | tr '\n' ' ')"

# ====================================================== RC 触发录包节点 ======
REC_LOG="$LOG_DIR/rc_record_${STAMP}.log"
if [ "$DO_RECORD" -eq 1 ]; then
  hr; log "启动 RC 触发录包节点：通道 6 拨到触发位开始录 bag（前缀 $BAG_PREFIX，目录 $BAG_DIR）"
  export RC_BAG_PREFIX="$BAG_PREFIX"
  export RC_BAG_DIR="$BAG_DIR"
  start_child "$PIDFILE_REC" "$REC_LOG" "$RC_RECORD_LAUNCHER"
  if wait_for "录包节点已注册（/rc_bag_recorder）" 30 \
      bash -c 'rosnode list 2>/dev/null | grep -qx /rc_bag_recorder'; then
    log "录包节点日志：$REC_LOG"
    if timeout 6 rostopic echo -n1 /rc_record/status 2>/dev/null | grep -q "state="; then
      ok "状态话题 /rc_record/status 正常"
    else
      warn "看不到 /rc_record/status，检查录包节点日志"
    fi
    have_rc="$(timeout 6 rostopic hz /mavros/rc/in 2>&1 | grep -c 'average rate')"
    if [ "${have_rc:-0}" -gt 0 ]; then
      ok "/mavros/rc/in 有数据（遥控链路正常）"
    else
      warn "看不到 /mavros/rc/in：遥控器未开机/未绑定/未校准，拨开关不会有任何反应"
    fi
  else
    err "录包节点未起来，看日志：$REC_LOG"
    stop_everything; exit 1
  fi
else
  hr; log "按要求未启动录包节点（--no-record）"
fi

echo
echo "=================================================================="
echo "  已就绪 —— 无桨测试可以开始"
echo "=================================================================="
echo "  包：$BAG_DIR/${BAG_PREFIX}_000001.bag（每拨一次新增一个序号）"
echo "  录包日志：$REC_LOG"
echo
cat <<'TIP'
遥控器操作：
  1) 确认通道 6 的极性与 ulog 同向：拨到「触发位」（PWM 大的那一侧）才开始录。
     现场确认：tail -f 上面的录包日志，会打印
     「通道 6 PWM=xxxx us -> 触发=True/False」；或 rostopic echo /mavros/rc/in 看 channels[5]。
     若拨下是低电平而你想「拨下即录」：在遥控器里把该通道反向（推荐），
     或改 scripts/rc_record/rc_record.yaml 的 active_high: false（但此时飞控 ulog 不会跟着录）。
  2) 拨下 -> bag 与 ulog 同时开始；拨回去后 1 s 收尾。
  3) 另一个终端观察：rostopic echo /rc_record/status  和  /mavros/state

无桨台架测试注意：
  * 卸桨后再解锁；解锁后 const_pos_mode 会变 False，属正常。
  * 解锁前后都看一眼 /mavros/estimator_status 的 pos_horiz_rel_status_flag 应为 True。
  * 电机转起来后留意 /mavros/local_position/pose 是否随振动抖动加剧。
  * 一次测试内不要重启链路：world 原点绑定 FAST-LIO 首次初始化，重启会跳变。
TIP
echo
echo "停止全部节点：按 Ctrl+C（录包节点会先把 bag 收尾写索引）"
echo "日志：$LOGFILE"
while true; do
  sleep 30
  st="$(timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | awk '/^armed:/{printf "armed=%s ", $2}')"
  pos="$(timeout 5 rostopic echo -n1 /mavros/local_position/pose 2>/dev/null | awk '/x:|y:|z:/{printf "%s ", $2}' | head -1)"
  rec="$(timeout 5 rostopic echo -n1 /rc_record/status 2>/dev/null | sed -n 's/^data: "\(state=[^ ]*\).*/\1/p')"
  log "运行中 · ${st}EKF xyz=[${pos}] · ${rec:-rec=?} · 在线节点 $(count_alive "$NODE_PAT")"
done
