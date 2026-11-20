#!/usr/bin/env bash
# ==============================================================================
# 融合快速启动（fusion）：在常驻 roscore / MAVROS 上，用几个单独的 roslaunch 拉起整条链路
#   定位核心（required，整组进退）：MID-360S 驱动 + FAST-LIO2 + 重力对齐桥接(位姿送飞控)
#   + 附属（RC 通道 6 触发录包自动重启、系留绕线器不重启）；MAVROS 开机常驻，不归本脚本起停
#   + 系留绕线器默认关闭（系留时使用 --winch 开启）
#   + OV13855 由监看面板或视觉任务按需启动
#
# 路径: /home/orangepi/mid360s_ws/scripts/fusion.sh
# launch: scripts/launch/ 下 fusion_core / fusion_aux（fusion.launch 是手动合集）
# 用法: ./fusion.sh [选项]
#   --no-record         不启动 RC 触发录包节点（本次不会有 bag）
#   --winch             启动绕线器控制（系留时显式开启）
#   --no-winch          不启动绕线器控制（默认，保留兼容）
#   --no-camera         保留兼容；相机由任务或网页按需启动
#   --bag-prefix NAME   包名前缀（默认 noprop，产出 NAME_000001.bag）
#   --bag-dir PATH      包输出目录（默认 /home/orangepi/test_data/bags）
#   --allow-unsynced-clock  不等对时，直接按板端本地时间启动（没外网时用）
#   --ev-mode MODE      外部定位送飞控的方式：pose（默认，只发位姿）/ odom（位姿 + 速度，要 MAVROS 用 MAVLink2）
#   -h|--help           帮助
#
# 目标：Wi-Fi 没连、断开、换网、重连都不让 FAST-LIO 停止、重启或重新初始化，飞控外部定位不因此中断。
#   1. 网络：ROS 核心用板内固定地址（dummy 网卡 ros0 10.250.32.1，master 走 127.0.0.1，见 ros_net.sh），
#      不跟随 Wi-Fi；本机 RViz 经到 10.250.32.1 的路由访问，换网只影响远程连接。
#   2. 时钟：启动前最多等 60 s 让 chrony 对时，没外网就按板端本地时间继续（标 UTC 未同步、不给飞控校时）；
#      核心起来前停掉 chronyd、收尾后再启动（clock_guard.sh），运行中系统时间不跳、不调速。
#   3. 进程：roscore 和 MAVROS 是开机常驻的 systemd 服务（scripts/resident/，mid360s-roscore / mid360s-mavros），
#      本脚本只用不起不停，fusion 没跑时也连着飞控（网页状态页靠它）。定位核心（fusion_core.launch）带 required，
#      任一节点退出整组停下、本脚本以失败退出、不自动重启（重启会重置 drone_odom 原点）；附属（fusion_aux.launch）
#      里的录包节点 respawn，退出只重启自己、不连带核心；绕线器不 respawn（auto_enable: true，重启会丢人工禁用 /
#      FAULT 锁存而重新自动使能），退出后只告警。MAVROS 退出由 systemd 重启；本脚本盯着 mavros_node 的 PID，
#      运行中变了就告警：重启后与飞控的时间同步要重新收敛，期间外部定位时间戳可能不准，看到告警应降落。
#      收尾时按环境变量标记（clock_guard.sh 的 clock_env）收拢 roslaunch 被强杀后留下的孤儿节点，全退了才恢复 chronyd。
#
# 前置检查：环境文件（ROS/工作空间 + rospack 找包）、常驻 roscore / MAVROS、防重复启动（锁 + 残留
# 进程/节点）、板内固定地址、板端时钟。之后用 fc_set_time.py 经 MAVROS 本地口把板端时间写进飞控 RTC
# （只影响 ulog 文件名/日期；板端没对上或飞控已解锁时跳过，失败只告警）。
# 不 ping 雷达、不拉取核对飞控参数、不逐话题测频、不等 EKF 就绪 —— 要逐项体检请用同目录
# start_handheld_test.sh；两个入口共用 /tmp/mid360s_fusion.lock，互相排斥。
#
# 手动启动（不经本脚本，没有时钟冻结；launch 里的注释指的就是这段；roscore / MAVROS 已常驻）：
#   cd /home/orangepi/mid360s_ws
#   source scripts/ros_net.sh && source /opt/ros/noetic/setup.bash && source devel/setup.bash
#   source /home/orangepi/orange/devel/setup.bash --extend
#   export ROS_PACKAGE_PATH="$PWD/scripts:$ROS_PACKAGE_PATH"   # rc_record 靠它定位
#   roslaunch --screen --required scripts/launch/fusion_core.launch      # 然后另开终端：
#   roslaunch --screen scripts/launch/fusion_aux.launch
#
# 前台运行。每个 roslaunch 都放进独立会话，信号只由本脚本统一转发：Ctrl+C / SIGTERM
# 按 相机 → 附属（录包节点先把 bag 收尾写索引）→ 核心 的顺序各发一次 SIGINT。
# 所有等待都是短轮询（≤ 3 s 一轮）并顺带检查核心是否还活着，所以启动/等首帧/稳态各阶段收到
# 信号都能及时收尾。收尾后校验本次生成的 bag 索引是否读得出来，最后按信号返回 130 / 143；
# 核心 roslaunch 自己退出时透传它的退出码，并在「没收到停止信号」时按异常返回 1（补掉 roslaunch
# 对 required 节点退出也返回 0 的坑）。本脚本不改飞控参数、不改源码、本身不做开机自启；
# 信号处理与退出码按后续接入 systemd 的需要保留。
#
# 附属起不来或中途退出、MAVROS 不在或重启都只告警，不影响核心与退出码。
# ==============================================================================
set -o pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/.." && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="$WS/devel/setup.bash"
ORANGE_SETUP="/home/orangepi/orange/devel/setup.bash"
CORE_LAUNCH="$DIR/launch/fusion_core.launch"
AUX_LAUNCH="$DIR/launch/fusion_aux.launch"
FC_TIME_PY="$DIR/fc_set_time.py"
RC_RECORD_DIR="$DIR/rc_record"
LOG_DIR="$WS/logs/fusion"
# 与 start_handheld_test.sh 共用同一把锁文件：新旧入口互斥
LOCK_FILE="/tmp/mid360s_fusion.lock"

# ROS 核心用板内固定地址（ros0 10.250.32.1，master 走 127.0.0.1），Wi-Fi 断开 / 换网不影响核心
source "$DIR/ros_net.sh" || exit 1
# 核心运行期间冻结板端时钟（停 chronyd，收尾再启动），见该文件头注释
source "$DIR/clock_guard.sh"
# 常驻 roscore / MAVROS 的服务名、MAVROS 本地口（FC_LOCAL_URL）和 mavros_pid / resident_proc
source "$DIR/resident/resident.sh"

NODES_TIMEOUT=90      # 等全部节点注册（秒）
READY_TIMEOUT=180     # 等首帧 /fastlio/odom_world（秒）
READY_POLL=3          # 等首帧时每轮阻塞上限（秒）：短轮询，好让信号能及时被处理
STOP_TIMEOUT=30       # 收尾时等 roslaunch 退出的上限（秒）
CLOCK_SYNC_TIMEOUT=60 # 等板端 chrony 对上时间（秒）；实测开机约 30 s 完成
CLOCK_MAX_OFFSET=0.05 # 对上之后允许的剩余偏差（秒），超过说明 chrony 还在大幅修正
CLOCK_SYNCED=0        # check_clock 确认板端已对上才置 1；飞控校时只在为 1 时写
FC_TIME_TIMEOUT=20    # 飞控校时脚本的总时限（秒），正常 5~10 s

DO_RECORD=1
ALLOW_UNSYNCED_CLOCK=0
DO_WINCH=0
BAG_PREFIX="noprop"
BAG_DIR="/home/orangepi/test_data/bags"
# 外部定位送飞控的方式（重力桥 ev_mode）：pose = /mavros/vision_pose/pose_cov 只发位姿；
# odom = /mavros/odometry/out 位姿 + 速度。启动时选定，运行中不切换（见 gravity_bridge.launch）。
EV_MODE="pose"

while [ $# -gt 0 ]; do
  case "$1" in
    --no-record) DO_RECORD=0 ;;
    --winch) DO_WINCH=1 ;;
    --no-winch) DO_WINCH=0 ;;
    --no-camera) : ;;
    --bag-prefix) BAG_PREFIX="${2:-}"; shift ;;
    --bag-dir) BAG_DIR="${2:-}"; shift ;;
    --allow-unsynced-clock) ALLOW_UNSYNCED_CLOCK=1 ;;
    --ev-mode) EV_MODE="${2:-}"; shift ;;
    -h|--help) sed -n '3,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$BAG_PREFIX" ] || { echo "包名前缀不能为空" >&2; exit 2; }
[ -n "$BAG_DIR" ] || { echo "包目录不能为空" >&2; exit 2; }
case "$EV_MODE" in pose|odom) ;; *) echo "--ev-mode 只能是 pose 或 odom（收到：$EV_MODE）" >&2; exit 2 ;; esac

mkdir -p "$LOG_DIR" "$BAG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_START_TS="$(date +%s)"        # 收尾时用它筛「本次生成的 bag」
LOGFILE="$LOG_DIR/fusion_${STAMP}.log"
# 本次运行专属目录：放 roslaunch 身份文件和 bag 清单，退出时整个删掉，
# 与上一次运行的文件彻底隔离，不会读到历史残留。
RUN_DIR="$(mktemp -d /tmp/mid360s_fusion.XXXXXX)"
CORE_PIDFILE="$RUN_DIR/core.pid"
AUX_PIDFILE="$RUN_DIR/aux.pid"
WATCH_PIDFILE="$RUN_DIR/watch.pid"
WATCH_CSV="$LOG_DIR/core_watch_${STAMP}.csv"

log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
ok()   { printf '[%s] [ OK ] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
warn() { printf '[%s] [WARN] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE"; }
err()  { printf '[%s] [FAIL] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOGFILE" >&2; }
hr()   { printf '%s\n' "----------------------------------------------------------------" | tee -a "$LOGFILE"; }

# 任何退出路径（正常、信号、启动失败）都放开时钟冻结：核心进程全退了 chronyd 才启动得了，
# 时间若有大偏差在这时一次校正
on_exit() {
  local _rc
  if [ "$CLOCK_FROZEN" -eq 1 ]; then
    clock_release; _rc=$?
    if [ "$_rc" -eq 0 ]; then
      log "chronyd 已恢复（核心已退出，时间若有大偏差在此时一次校正）"
    elif [ "$_rc" -eq 2 ]; then
      warn "本次启动的进程还有残留（pid $(tagged_pids | tr '\n' ' ')），chronyd 保持停止、防启动检查也会挡住它"
      warn "处理掉这些进程后执行 sudo systemctl start chrony"
    else
      warn "chronyd 没起来：多半还有核心进程占着 $CLOCK_FREEZE_LOCK，等它们退出后执行 sudo systemctl start chrony"
    fi
  fi
  rm -rf "$RUN_DIR"
}
trap on_exit EXIT

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

# 常驻服务里的进程（roscore、MAVROS 的 roslaunch / mavros_node）不算残留，按 cgroup 排除
alive() {
  for _pid in $(pgrep -f "$1" 2>/dev/null); do
    case "$SELF_PIDS" in *" $_pid "*) continue ;; esac
    resident_proc "$_pid" && continue
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

CORE_SETSID_PID=""    # 核心 roslaunch 的 setsid --wait 包装进程：最后 wait 它拿退出码
SHUTTING_DOWN=0
MISSING_NODES=""
FC_TIME_PID=""        # 飞控校时脚本（timeout 包装）的 PID，收尾时停掉它（它会先关掉飞控上的 MAVLink shell）
MAVROS_PID0=""        # 核心启动时常驻 mavros_node 的 PID；运行中变了说明 MAVROS 重启过
MAVROS_SEEN=""        # 已经告警过的 mavros_node PID（含空串＝不在运行），同一状态只报一次

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

# 包装进程先把「自己的 PID + 启动时刻」写进 pidfile 再 exec：exec 不改这两项，所以收尾时读到的
# 身份依然有效，可用来核对、防 PID 复用（进程早已退出、PID 被别的进程复用时绝不误停）。
SPAWN_WRAPPER='
  _pf="$1"; shift; _cmd=("$@")
  _s="$(cat /proc/$$/stat 2>/dev/null)"; _s="${_s##*)}"
  set -- $_s
  printf "%s %s\n" "$$" "${20:-}" >"$_pf"
  exec "${_cmd[@]}"
'
# spawn PIDFILE OUT CMD...：放进独立会话（setsid）后台运行，终端信号不会绕过本脚本直接打到它，
# 由收尾统一发一次 SIGINT，避免重复信号打断 roslaunch 的收尾流程。OUT=tee 时输出进终端和主日志
# （tee 忽略 SIGINT：否则 Ctrl+C 会先杀掉 tee，roslaunch 再写日志就吃 SIGPIPE），否则追加进该文件。
# SPAWN_PID 是 setsid --wait 包装进程（wait 它拿得到命令的退出码）；等 pidfile 落地最多 12 s。
# 冻结时钟的锁（fd 201）随之被继承；另按 pidfile 名（core / aux …）打上 clock_env 环境变量标记，
# 一路继承到 roslaunch 用 setsid 起的节点：roslaunch 被强杀留下的孤儿节点收尾时据此找回来。
spawn() {
  local _pf="$1" _out="$2" _i=0
  shift 2
  rm -f "$_pf"
  : >"$_pf.tried"   # 先置位再 fork：这段窗口里收到信号，收尾也知道「已经发起过」
  # shellcheck disable=SC2046
  if [ "$_out" = tee ]; then
    env $(clock_env "$(basename "$_pf" .pid)") setsid --wait bash -c "$SPAWN_WRAPPER" _ "$_pf" "$@" \
      </dev/null > >( trap '' INT; exec tee -a "$LOGFILE" ) 2>&1 &
  else
    env $(clock_env "$(basename "$_pf" .pid)") setsid --wait bash -c "$SPAWN_WRAPPER" _ "$_pf" "$@" \
      </dev/null >>"$_out" 2>&1 &
  fi
  SPAWN_PID=$!
  while [ ! -s "$_pf" ] && [ "$_i" -lt 60 ]; do
    sleep 0.2; _i=$((_i + 1))
  done
  [ -s "$_pf" ]
}

# 按 pidfile 核对身份（PID + 启动时刻）后输出 PID；进程不在或 PID 已被复用返回 1
proc_pid_now() {
  local _pid _st _now
  read -r _pid _st <"$1" 2>/dev/null || return 1
  [ -n "$_pid" ] || return 1
  kill -0 "$_pid" 2>/dev/null || return 1
  if [ -n "$_st" ]; then
    _now="$(proc_starttime "$_pid")" || return 1
    [ "$_now" = "$_st" ] || return 1
  fi
  printf '%s\n' "$_pid"
}
proc_alive() { proc_pid_now "$1" >/dev/null 2>&1; }
core_alive() { proc_alive "$CORE_PIDFILE"; }

# stop_proc 名称 PIDFILE 超时秒：只停本次启动的那个进程（按 pidfile 核对身份，绝不按进程名扫描）。
# SIGINT 让 roslaunch 按顺序收拢自己的节点；启动瞬间那一下可能落在「fork 完、还没 exec」的窗口里
# 被丢掉（后台任务的子进程在 exec 前忽略 SIGINT），所以每 5 s 补一次 —— 重复 SIGINT 不会打断
# roslaunch 的收尾（实测连发 3 次仍然无残留、bag 索引完好）。超时 SIGTERM 兜底。没起过的直接跳过。
# 主进程退了（或本来就已被强杀）之后，再按环境变量标记收拢这一组留下的孤儿节点：roslaunch 用 setsid
# 起节点，roslaunch 一死节点不会跟着退。
stop_proc() {
  local _name="$1" _pf="$2" _limit="$3" _i=0
  [ -e "$_pf.tried" ] || return 0
  while [ ! -s "$_pf" ] && [ "$_i" -lt 30 ]; do
    sleep 0.1; _i=$((_i + 1))
  done
  stop_leader "$_name" "$_pf" "$_limit"; _i=$?
  reap_tagged "$(basename "$_pf" .pid)" "$_name" 2>&1 | tee -a "$LOGFILE"
  [ "${PIPESTATUS[0]}" -eq 0 ] && return "$_i"
  return 1
}
stop_leader() {
  local _name="$1" _pf="$2" _limit="$3" _pid _cur _i=0
  _pid="$(proc_pid_now "$_pf")" || { log "$_name 已不在运行"; return 0; }
  log "SIGINT -> $_name $_pid"
  kill -INT "$_pid" 2>/dev/null || true
  _i=0
  while [ "$_i" -lt $((_limit * 2)) ]; do
    proc_alive "$_pf" || return 0
    sleep 0.5; _i=$((_i + 1))
    if [ $((_i % 10)) -eq 0 ]; then
      _cur="$(proc_pid_now "$_pf")" && kill -INT "$_cur" 2>/dev/null
    fi
  done
  warn "$_name 未在 ${_limit} s 内退出，SIGTERM 兜底"
  _cur="$(proc_pid_now "$_pf")" && kill -TERM "$_cur" 2>/dev/null
  _i=0
  while [ "$_i" -lt 10 ]; do
    proc_alive "$_pf" || return 0
    sleep 0.5; _i=$((_i + 1))
  done
  warn "$_name 仍在运行（pid=$_pid），需手动 kill -INT 处理"
  return 1
}

# 收尾顺序：相机 → 附属（录包节点先把 bag 收尾写索引）→ 核心 → 监控。
# roscore 和 MAVROS 是常驻服务，不停。
stop_all() {
  stop_proc "附属 roslaunch（录包节点会先把 bag 收尾）" "$AUX_PIDFILE" "$STOP_TIMEOUT"
  stop_proc "核心 roslaunch" "$CORE_PIDFILE" "$STOP_TIMEOUT"
  stop_proc "数据连续性监控（退出时打印最终汇总）" "$WATCH_PIDFILE" 10
}

# 常驻 MAVROS 的状态，运行中每轮调用：mavros_node 不在或 PID 跟核心启动时不同就告警，同一状态只报一次。
# MAVROS 重启后与飞控的时间同步要重新收敛，期间送给飞控的外部定位时间戳可能不准。
check_mavros() {
  local _p
  _p="$(mavros_pid)"
  [ "$_p" = "$MAVROS_SEEN" ] && return 0
  MAVROS_SEEN="$_p"
  if [ -z "$_p" ]; then
    warn "常驻 MAVROS 不在运行（systemd 会重启它）：飞控暂时收不到外部定位"
  elif [ -z "$MAVROS_PID0" ]; then
    warn "常驻 MAVROS 起来了（pid $_p）：时间同步刚开始收敛，起飞前确认 EKF 已接受外部定位"
  elif [ "$_p" != "$MAVROS_PID0" ]; then
    warn "常驻 MAVROS 重启过（pid ${MAVROS_PID0:-无} → $_p）：时间同步要重新收敛，外部定位时间戳可能不准，在飞应降落"
  fi
}


# ------------------------------------------------------------------- 收尾 ------
# 收尾检查：只报告，不再顺手杀别的进程
verify_clean() {
  hr; log "收尾检查"
  local _left=""
  alive "livox_ros_driver2_nod[e]" && _left="$_left livox_ros_driver2_node"
  alive "fastlio_mappin[g]"      && _left="$_left fastlio_mapping"
  alive "gravity_bridge_nod[e]"  && _left="$_left gravity_bridge_node"
  alive "bag_recorder_node.p[y]" && _left="$_left bag_recorder_node.py"
  alive "core_watch.p[y]"        && _left="$_left core_watch.py"
  if [ "$DO_WINCH" -eq 1 ] && alive "tether_winch_nod[e]"; then
    _left="$_left tether_winch_node"
  fi
  if [ -n "$_left" ]; then
    warn "仍有节点进程残留：$_left"
    warn "核对：ps -eo pid,args | grep -E 'livox_ros_driver2_node|fastlio_mapping|gravity_bridge_node|bag_recorder_node|tether_winch_node'"
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

  # roscore / MAVROS 本来就该常驻，只报一下它们还在
  if unit_active "$RESIDENT_ROSCORE_UNIT" && [ -n "$(mavros_pid)" ]; then
    ok "常驻 roscore / MAVROS 照常运行"
  else
    warn "常驻服务有不在的：systemctl status $RESIDENT_ROSCORE_UNIT $RESIDENT_MAVROS_UNIT"
  fi
}

on_signal() {
  local _sig="$1" _code="$2" _i=0
  if [ "$SHUTTING_DOWN" -eq 1 ]; then
    warn "收尾进行中，忽略重复的 SIG$_sig"
    return
  fi
  SHUTTING_DOWN=1
  hr; warn "收到 SIG$_sig：只收拢本次启动的 roslaunch（常驻 roscore / MAVROS 不停）"
  if [ -n "$FC_TIME_PID" ]; then
    # timeout 把 SIGTERM 转给校时脚本，脚本先关掉飞控上的 MAVLink shell 再退出（< 1 s）；
    # 等它退干净，免得 shell 一直开在飞控上
    log "停止飞控校时脚本（pid=$FC_TIME_PID）"
    kill -TERM "$FC_TIME_PID" 2>/dev/null
    while kill -0 "$FC_TIME_PID" 2>/dev/null && [ "$_i" -lt 15 ]; do
      sleep 0.2; _i=$((_i + 1))
    done
  fi
  stop_all
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
hr; log "步骤 1/3：前置检查（环境 / 常驻 roscore 与 MAVROS / 防重复启动 / 板端时钟）"

[ -f "$ROS_SETUP" ] || { err "缺少 $ROS_SETUP"; exit 1; }
[ -f "$WS_SETUP" ]  || { err "缺少 $WS_SETUP，请先在板端 catkin build"; exit 1; }
for f in "$CORE_LAUNCH" "$AUX_LAUNCH"; do
  [ -f "$f" ] || { err "缺少 $f"; exit 1; }
done
[ -f "$RC_RECORD_DIR/start_rc_record.sh" ] || { err "缺少 $RC_RECORD_DIR/start_rc_record.sh"; exit 1; }
[ -f "$RC_RECORD_DIR/package.xml" ] || { err "缺少 $RC_RECORD_DIR/package.xml（roslaunch 靠它定位 rc_record 包）"; exit 1; }
ok "脚本、launch 与 rc_record 包文件齐全（工作空间 $WS）"

set +u; source "$ROS_SETUP"; source "$WS_SETUP"; set -u
if [ "$DO_WINCH" -eq 1 ]; then
  [ -f "$ORANGE_SETUP" ] || { err "缺少 $ORANGE_SETUP；请构建 orange 或使用 --no-winch"; exit 1; }
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
ok "ROS Noetic 与所选功能包齐备"

# roscore / MAVROS 是开机常驻服务（scripts/resident/install.sh 装的），这里只确认它们在。
# master 起不来就不启动；MAVROS 不在或没连上飞控只告警（和以前 MAVROS 起不来一样，核心照常运行）。
if ! unit_active "$RESIDENT_ROSCORE_UNIT" || ! timeout 5 rosnode list >/dev/null 2>&1; then
  err "常驻 roscore 不可达：systemctl status $RESIDENT_ROSCORE_UNIT（没装过就跑 $DIR/resident/install.sh）"
  exit 1
fi
ok "常驻 roscore 可达（master 走 127.0.0.1:11311）"
# 常驻 master 的参数会一直留着：离线回放之类留下 /use_sim_time=true，核心节点就会干等 /clock
if [ "$(timeout 5 rosparam get /use_sim_time 2>/dev/null)" = "true" ]; then
  err "常驻 master 上 /use_sim_time=true（多半是回放留下的）：rosparam set /use_sim_time false 后再启动"
  exit 1
fi
if [ -z "$(mavros_pid)" ]; then
  warn "常驻 MAVROS 不在运行（systemctl status $RESIDENT_MAVROS_UNIT）：核心照常起，飞控收不到外部定位"
elif timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q "connected: True"; then
  ok "常驻 MAVROS 已连上飞控（mavros_node pid $(mavros_pid)）"
else
  warn "常驻 MAVROS 在跑但没连上飞控（飞控没上电？）：核心照常起，飞控收不到外部定位"
fi
# odom 方式发 ODOMETRY（MAVLink2 才有的消息）：MAVROS 还是 v1.0 时会把它丢掉，飞控什么都收不到
if [ "$EV_MODE" = "odom" ] && [ -n "$(mavros_pid)" ]; then
  _proto="$(timeout 5 rosparam get /mavros/fcu_protocol 2>/dev/null)"
  if [ "$_proto" != "v2.0" ]; then
    err "--ev-mode odom 要 MAVROS 用 MAVLink2，常驻 MAVROS 是 fcu_protocol=${_proto:-读不到}"
    err "改 scripts/launch/mavros.launch 的 fcu_protocol 后 sudo systemctl restart $RESIDENT_MAVROS_UNIT，或改用 --ev-mode pose"
    exit 1
  fi
  ok "常驻 MAVROS 用 MAVLink2（odom 方式可用）"
fi

stale=""
for pat in "roslaunc[h]" "livox_ros_driver2_nod[e]" \
           "fastlio_mappin[g]" "gravity_bridge_nod[e]" "bag_recorder_node.p[y]" "core_watch.p[y]"; do
  alive "$pat" && stale="$stale ${pat//[\[\]]/}"
done
if [ "$DO_WINCH" -eq 1 ] && alive "tether_winch_nod[e]"; then
  stale="$stale tether_winch_node"
fi
if [ -n "$stale" ]; then
  err "检测到残留进程：$stale"
  err "上一次实例可能没退干净，先收尾（Ctrl+C 或 kill -INT <roslaunch pid>）再启动"
  exit 1
fi
ok "无残留 ROS 进程（常驻服务除外）"

_nodes="$(timeout 5 rosnode list 2>/dev/null)"
_busy=""
_check="${EXPECT_NODES#/mavros } /rc_bag_recorder"   # /mavros 是常驻的，本来就该在
for n in $_check; do
  printf '%s\n' "$_nodes" | grep -qx "$n" && _busy="$_busy $n"
done
if [ -n "$_busy" ]; then
  err "ROS Master 上已挂着本链路节点：$_busy"
  err "先停掉旧实例再启动"
  exit 1
fi
ok "ROS Master 上没有本链路节点"

# 板端 RTC 掉电不保持：开机先用 fake-hwclock 存的旧时间，chrony 连上外网时间源后
# 再一次性跳到正确时间（makestep 1 3；实测开机约 30 s 时跳了 +1045 s）。MID-360S 没接
# PTP，雷达/IMU 时间戳就是板端系统时间：链路运行中时间一跳，往前跳 FAST-LIO 的 IMU
# 积分会出错，往回跳桥接直接 fatal。所以启动前先给 chrony 最多 CLOCK_SYNC_TIMEOUT 对时；
# 没外网对不上就按板端本地时间继续（标 UTC 未同步、不给飞控校时），核心起来前再停掉
# chronyd（clock_freeze），运行中不论何时联网都不会跳，收尾后才恢复对时。
# 每轮只阻塞 ≤ 3 s，等待期间 Ctrl+C 照样能及时处理；计时用 /proc/uptime（开机以来的
# 单调时间），不受等待期间时钟跳变影响。
clock_status() {  # 输出 "<Leap status> <剩余偏差绝对值(秒)>"；chronyd 不可达时返回非 0
  local _t
  _t="$(timeout 3 chronyc -n tracking 2>/dev/null)" || return 1
  printf '%s\n' "$_t" | awk -F': *' '
    /^Leap status/ { leap = $2; gsub(/ /, "_", leap) }
    /^System time/ { split($2, a, " "); off = a[1] + 0; if (off < 0) off = -off }
    END { if (leap == "" || off == "") exit 1; printf "%s %.6f\n", leap, off }'
}
uptime_s() { awk '{ printf "%d\n", $1 }' /proc/uptime; }
check_clock() {
  local _t0="" _leap="" _off="" _why="" _waiting=0 _limit="$CLOCK_SYNC_TIMEOUT"
  [ "$ALLOW_UNSYNCED_CLOCK" -eq 1 ] && _limit=0
  if ! command -v chronyc >/dev/null 2>&1; then
    _why="找不到 chronyc"
  else
    # 上次异常退出时 chronyd 可能还停着（锁被残留进程占着），先拉起来再对时
    clock_ensure_running || warn "chronyd 没在运行，也没能启动"
    _t0="$(uptime_s)"
    while :; do
      if read -r _leap _off < <(clock_status); then
        if [ "$_leap" = "Not_synchronised" ]; then
          _why="chrony 还没同步上时间源"
        elif awk -v o="$_off" -v m="$CLOCK_MAX_OFFSET" 'BEGIN { exit !(o < m) }'; then
          ok "板端时间已由 chrony 对上（剩余偏差 ${_off} s）"
          CLOCK_SYNCED=1
          return 0
        else
          _why="chrony 已同步，但剩余偏差 ${_off} s 超过 ${CLOCK_MAX_OFFSET} s，还在修正"
        fi
      else
        _why="chronyd 无响应"
      fi
      [ $(( $(uptime_s) - _t0 )) -ge "$_limit" ] && break
      if [ "$_waiting" -eq 0 ]; then
        _waiting=1
        log "板端时间还没对上（${_why}），等 chrony 对时，最多 ${CLOCK_SYNC_TIMEOUT} s"
      fi
      sleep 2
    done
  fi
  # 对不上也启动：运行中时钟冻结（clock_freeze），不会因为后来联网而跳
  warn "板端 UTC 时间未同步（${_why}），按板端本地时间启动"
  warn "运行中时钟冻结、不会跳；但 bag / 日志的绝对时间可能不准，本次不给飞控校时"
  [ "$ALLOW_UNSYNCED_CLOCK" -eq 1 ] || warn "确定没外网时可加 --allow-unsynced-clock，跳过这 ${CLOCK_SYNC_TIMEOUT} s 等待"
  return 0
}
check_clock || exit 1
# 等对时期间时钟可能已经跳过，收尾筛「本次生成的 bag」的起点按对时之后重取
RUN_START_TS="$(date +%s)"

# ============================================================== 飞控校时 =====
# Pixhawk 6C 的 RTC 断电停走、开机接着走（每断一次电慢一个断电时长），PX4 又只在自身
# 时钟早于 2009 年时才接受 MAVROS 的 SYSTEM_TIME，所以由 fc_set_time.py 经常驻 MAVROS 的本地口
# （FC_LOCAL_URL，串口一直被 MAVROS 占着）走 MAVLink shell 执行 date -s 写进飞控 RTC。只影响 ulog
# 文件名/日期，不影响融合，所以任何情况都只告警、不拦启动。板端没对上（--allow-unsynced-clock 放行）
# 时不写，免得把错误时间带给飞控。
sync_fc_time() {
  local _pid _rc _line
  hr; log "飞控校时：经 MAVROS 本地口把板端时间写进飞控 RTC"
  if [ "$CLOCK_SYNCED" -ne 1 ]; then
    warn "板端时间没对上，跳过飞控校时（本次 ulog 文件名时间可能不准）"
    return 0
  fi
  if [ ! -f "$FC_TIME_PY" ]; then
    warn "缺 $FC_TIME_PY，跳过飞控校时"
    return 0
  fi
  if [ -z "$(mavros_pid)" ]; then
    warn "常驻 MAVROS 不在运行，跳过飞控校时"
    return 0
  fi
  # 放后台再 wait：等待期间收到 Ctrl+C/SIGTERM 能立即进 on_signal（此时 roslaunch 还没起，
  # 脚本自带总时限，到时自己退出并关掉飞控上的 shell）
  timeout "$FC_TIME_TIMEOUT" python3 "$FC_TIME_PY" --dev "$FC_LOCAL_URL" \
    >"$RUN_DIR/fc_time.out" 2>&1 &
  _pid=$!
  FC_TIME_PID="$_pid"
  wait "$_pid"; _rc=$?
  FC_TIME_PID=""
  while IFS= read -r _line; do
    [ -n "$_line" ] && log "  $_line"
  done <"$RUN_DIR/fc_time.out"
  case "$_rc" in
    0)   ok "飞控时间与板端对齐（误差 1 s 内）" ;;
    2)   warn "飞控已解锁，跳过飞控校时" ;;
    3)   warn "缺 pymavlink，跳过飞控校时（python3 -m pip install --user pymavlink）" ;;
    124) warn "飞控校时超过 ${FC_TIME_TIMEOUT} s 没完成，跳过（本次 ulog 文件名时间可能不准）" ;;
    *)   warn "飞控校时失败（退出码 $_rc），不影响启动（本次 ulog 文件名时间可能不准）" ;;
  esac
  return 0
}
sync_fc_time

# ============================================================== 启动 =========
hr; log "步骤 2/3：在常驻 roscore / MAVROS 上启动核心 / 附属（scripts/launch/）"
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
  log "绕线器：关闭（默认，--winch 可开启），不由 fusion 发送 DUTY"
fi
if [ "$EV_MODE" = "odom" ]; then
  log "外部定位：odom 方式，位姿 + 速度送 /mavros/odometry/out（飞控 EKF2_EV_CTRL 带 bit2 才融合速度）"
else
  log "外部定位：pose 方式，只送位姿到 /mavros/vision_pose/pose_cov"
fi

# 核心起来前冻结时钟：停掉 chronyd，运行中不论何时联网、chrony 被谁重启都不会跳时间；
# 冻结锁（fd 201）会被下面的 roslaunch 继承，收尾时 on_exit 再恢复。冻结不了就不启动。
clock_freeze; _rc=$?
if [ "$_rc" -eq 1 ]; then
  err "时钟冻结锁 $CLOCK_FREEZE_LOCK 被占用：还有别的定位核心实例在运行"
  exit 1
elif [ "$_rc" -ne 0 ]; then
  err "停不掉 chronyd（/etc/sudoers.d/mid360s-chrony 没装？），运行中可能跳时间，不启动"
  exit 1
fi
ok "已停 chronyd：运行中系统时钟不跳、不调速，收尾后恢复对时"

# 启动顺序：监控 → 核心 → 附属，各自一个独立会话（spawn），身份按 pidfile 核对。
# 核心起不来就收掉已起的进程、以失败退出；附属起不来只告警，核心照常运行。
start_fail() {
  SHUTTING_DOWN=1
  err "$1"; err "看日志：$LOGFILE"
  stop_all; verify_clean
  exit 1
}

# 记下本次 UTC 是否对上：回放 / 分析 bag 时据此判断绝对时间可不可信
if [ "$CLOCK_SYNCED" -eq 1 ]; then
  rosparam set /fusion/utc_synced true
else
  rosparam set /fusion/utc_synced false
  warn "已置 /fusion/utc_synced=false：本次 bag / 日志按板端本地时间"
fi
# 数据连续性监控先于核心起：从第一帧开始记。只读消息头，断流 / 时间戳倒退 / 时间跳变 / 飞控本地位置与
# 视觉位姿偏差即时打到终端，逐秒明细进 CSV。起不来只告警。
if spawn "$WATCH_PIDFILE" tee python3 -u "$DIR/core_watch.py" --csv "$WATCH_CSV" --ev-mode "$EV_MODE"; then
  log "数据连续性监控 pid=$(proc_pid_now "$WATCH_PIDFILE")（逐秒明细：$WATCH_CSV）"
else
  warn "数据连续性监控没起来（核心照常运行）"
fi

spawn "$CORE_PIDFILE" tee roslaunch --screen --required "$CORE_LAUNCH" send_to_mavros:=true \
  ev_mode:="$EV_MODE" || start_fail "核心 roslaunch 没能在 12 s 内起来"
CORE_SETSID_PID="$SPAWN_PID"
log "核心 roslaunch pid=$(proc_pid_now "$CORE_PIDFILE")（雷达驱动 / FAST-LIO / 重力桥，任一退出整组停）"
# 从这里起盯常驻 MAVROS：mavros_node 的 PID 变了就是重启过（check_mavros）
MAVROS_PID0="$(mavros_pid)"
MAVROS_SEEN="$MAVROS_PID0"
log "常驻 MAVROS：mavros_node pid=${MAVROS_PID0:-不在运行}（运行中重启会告警）"
if [ "$DO_RECORD" -eq 1 ] || [ "$DO_WINCH" -eq 1 ]; then
  if spawn "$AUX_PIDFILE" tee roslaunch --screen "$AUX_LAUNCH" \
       "record:=$REC_ARG" "winch:=$WINCH_ARG" bag_prefix:="$BAG_PREFIX" bag_dir:="$BAG_DIR"; then
    log "附属 roslaunch pid=$(proc_pid_now "$AUX_PIDFILE")（录包节点退出自动重启；绕线器退出不重启、只告警）"
  else
    warn "附属 roslaunch 没能在 12 s 内起来：录包 / 绕线器不可用（核心照常运行）"
  fi
fi
log "节点日志同时进终端和 $LOGFILE"

# ============================================================== 就绪 =========
hr; log "步骤 3/3：等待节点注册与桥接输出"
T0=$SECONDS
wait_for_nodes() {
  local _t=0 _cur _miss _n
  while [ "$_t" -lt "$NODES_TIMEOUT" ]; do
    core_alive || return 2
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
  err "核心 roslaunch 在节点注册完成前就退出了 —— 启动失败，看上面的日志"
elif [ "$NODES_RC" -eq 0 ]; then
  ok "节点已启动（耗时 $((SECONDS - T0)) s）：$EXPECT_NODES"
  if [ "$DO_RECORD" -eq 1 ]; then
    if timeout 5 rosnode list 2>/dev/null | grep -qx /rc_bag_recorder; then
      ok "录包节点已注册 /rc_bag_recorder（通道 6 拨到触发位开始录）"
    else
      warn "录包节点没注册上，检查上面的日志"
    fi
  fi
  hr; log "等待桥接输出里程计：首帧 /fastlio/odom_world（桥接收到有效 gravity_init 才发）"
  # 每轮只阻塞 READY_POLL 秒：Bash 要等前台命令结束才处理 trap，
  # 一次 timeout 180 的同步等待会让 SIGTERM 拖到 180 s，期间节点崩了也报不出来。
  _t=0; _ready=0
  while [ "$_t" -lt "$READY_TIMEOUT" ]; do
    if timeout "$READY_POLL" rostopic echo -n1 /fastlio/odom_world >/dev/null 2>&1; then
      _ready=1; break
    fi
    core_alive || break
    check_mavros
    _t=$((_t + READY_POLL))
  done
  if [ "$_ready" -eq 1 ]; then
    ok "桥接已输出里程计（从启动到首帧共 $((SECONDS - T0)) s）"
    # odom 方式要位姿和速度配上才送飞控：FAST-LIO 没发 /fastlio/velocity 时这里收不到
    if [ "$EV_MODE" = "odom" ]; then
      if timeout 5 rostopic echo -n1 /fastlio/vision_odom >/dev/null 2>&1; then
        ok "odom 方式：位姿 + 速度已在送 /mavros/odometry/out"
      else
        warn "odom 方式 5 s 内没有 /fastlio/vision_odom：FAST-LIO 没发 /fastlio/velocity？飞控收不到外部定位，看桥接日志"
      fi
    fi
    log "注意：这只说明 FAST-LIO→桥接这一段通了。MAVROS 有没有连上飞控、PX4 有没有"
    log "      接受外部定位，本入口不检查（要验收请用 start_handheld_test.sh）"
  elif ! core_alive; then
    warn "等首帧期间核心 roslaunch 已经退出（见上面的日志）"
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
if core_alive; then
  echo
  echo "=================================================================="
  echo "  链路运行中 —— 停止：Ctrl+C"
  echo "=================================================================="
  echo "  包：$BAG_DIR/${BAG_PREFIX}_000001.bag（每拨一次新增一个序号）"
  echo "  日志：$LOGFILE"
  echo "  核心 / 附属都在独立会话里，收尾只能靠本脚本；roscore / MAVROS 常驻，收尾不停"
  echo "  本机 RViz：板内地址 $ROS_IP，经板端 Wi-Fi ${ROS_NET_WIFI_IP:-（未连接）} 转发"
  echo
  if [ "$DO_RECORD" -eq 1 ]; then
    cat <<'TIP'
遥控器：
  1) 通道 6 拨到「触发位」（PWM 大的那一侧）开始录 bag，拨回去 1 s 后收尾。
  2) 现场确认极性：rostopic echo /mavros/rc/in 看 channels[5]，
     或另开终端 rostopic echo /rc_record/status 看 state=REC/IDLE。
  3) 飞控 ulog 同向（SDLOG_MODE=3 + RC_MAP_AUX1=6）时，同一个开关同时录 bag 与 ulog。
  4) 一次测试内不要重启链路：drone_odom 原点绑定 FAST-LIO 首次初始化，重启会跳变。
TIP
    echo
  fi
  HB=0
  _utc="utc=ok"; [ "$CLOCK_SYNCED" -eq 1 ] || _utc="utc=未同步"
  WINCH_GONE=0
  while core_alive; do
    sleep 2
    HB=$((HB + 2))
    check_mavros
    # 绕线器不 respawn（见 fusion_aux.launch）：退出后立即告警一次
    if [ "$DO_WINCH" -eq 1 ] && [ "$WINCH_GONE" -eq 0 ] && ! alive "tether_winch_nod[e]"; then
      WINCH_GONE=1
      warn "绕线器节点已退出且不会自动重启（重启会丢人工禁用 / FAULT 锁存、重新自动使能）：人工检查后再重启 fusion"
    fi
    if [ "$HB" -ge 30 ]; then
      HB=0
      _armed="$(timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | sed -n 's/^armed: //p' | head -1)"
      _rec="$(timeout 5 rostopic echo -n1 /rc_record/status 2>/dev/null | sed -n 's/^data: "\(state=[^ ]*\).*/\1/p')"
      if [ -z "$MAVROS_SEEN" ]; then _mav="mavros=down"
      elif [ "$MAVROS_SEEN" != "$MAVROS_PID0" ]; then _mav="mavros=重启过"
      else _mav="mavros=up"; fi
      _aux=""
      if [ -e "$AUX_PIDFILE.tried" ]; then
        if proc_alive "$AUX_PIDFILE"; then _aux=" · aux=up"; else _aux=" · aux=down"; fi
      fi
      _winch=""
      if [ "$DO_WINCH" -eq 1 ]; then
        if [ "$WINCH_GONE" -eq 0 ]; then _winch=" · winch=up"; else _winch=" · winch=已退出"; fi
      fi
      log "运行中 · armed=${_armed:-?} · ${_mav}${_aux}${_winch} · ${_rec:-rec=n/a} · ${_utc}"
    fi
  done
fi

# ============================================================== 退出码 =======
if [ -n "$CORE_SETSID_PID" ]; then
  wait "$CORE_SETSID_PID"; RL_RC=$?
else
  RL_RC=0
fi
hr
# 能走到这里说明没收到过停止信号（收到信号的都在 on_signal 里 exit 掉了），
# 所以核心 roslaunch 是自己退的：required 之下就是雷达驱动 / FAST-LIO / 重力桥某个先退出，
# 把整组带下来了。不自动重启（会重置 drone_odom 原点），收掉其余进程后以失败退出。
# roslaunch 遇到 required 节点退出也给退出码 0，这里补一个非 0，
# 免得 systemd / 上层脚本把「节点崩了」当成「正常结束」。
if [ "$RL_RC" -ne 0 ]; then
  err "核心 roslaunch 异常退出（退出码 $RL_RC），往上翻日志里的第一个报错"
  EXIT_RC="$RL_RC"
elif [ "$SHUTTING_DOWN" -eq 0 ]; then
  err "核心 roslaunch 在没收到停止信号的情况下自行退出：飞控已收不到外部定位"
  err "多半是某个核心节点先退出触发了 required 全体收拢，找日志里的 \"REQUIRED process [...] has died!\" 确认是哪个"
  EXIT_RC=1
else
  ok "核心 roslaunch 已退出（退出码 0）"
  EXIT_RC=0
fi
SHUTTING_DOWN=1
stop_all
verify_clean
log "退出（日志：$LOGFILE）"
exit "$EXIT_RC"
