# shellcheck shell=bash
# ==============================================================================
# 导航实飞测试的公共部分：由 scripts/nav_testN.sh source，不单独运行。
# 每个测试脚本定义 TEST_NAME、test_desc（一行说明）、test_arg（自己的选项）、run_test（测试动作），再调 nav_test_main "$@"。
#
# 公共选项：
#   --preview          只预览：导航不发飞控；旧测试膨胀 0.2 / 0.15 m，indoor1 保持实飞 0.4 / 0.3 m
#   --map DIR          无人机地图（各测试指定默认值；旧测试默认室外图）
#   --reloc-config F   重定位配置（各测试指定默认值；旧测试默认 outdoor.yaml）
#   --map-frame F      world map 标定（各测试指定默认值；旧测试默认 map_frame_outdoor.yaml）
#   --nav-config F     导航配置（各测试指定默认值；旧测试默认 outdoor_nav.yaml）
#   --show-dll-config   只显示本次任务的旁路与录包配置，随后退出
#   --no-dll            本次跳过 DLL 旁路对照
#   --record-raw-lidar  本次额外录原始 /livox/lidar，供重新回放和静止 ICP
#   --ev-mode MODE     转给 fusion.sh：外部定位送飞控的方式，pose（默认，只发位姿）/ odom（位姿 + 速度，
#                      要 MAVROS 用 MAVLink2、飞控 EKF2_EV_CTRL 带速度位 bit2）。只能开机时选，测试中途不能换
#   --live-grid-only   跳过先验栅格，仅用当前测试的实时观测
#   --allow-unsynced-clock  转给 fusion.sh：不等对时（没外网时用）
#   -h|--help          帮助
#
# 流程：检查 → 载入地图缓存 → fusion.sh（雷达 + FAST-LIO + 重力桥送飞控 + RC 通道 6 录包）
#   → 重定位 + car_bridge（world map）→ 导航（EGO + 网关）→ run_test（各测试自己定义）→ 等 Ctrl+C 收尾。
# 从起 fusion 到「已定位」飞机放在起飞点别动；之后才解锁起飞。
# 飞机还在解锁状态时 Ctrl+C 不收尾（停链路会断飞控的外部定位），降落上锁后再 Ctrl+C。
# 不在 tmux 里会自己开 tmux 会话 nav：ssh 断了也接着跑，回来 tmux attach -t nav。
# 每次的日志、RC 包（bag 和飞控 ulog 都由通道 6 触发）、目标记录在 ~/test_data/nav_test/<时间>_<测试名>/。
# ==============================================================================
set -o pipefail

NT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIR="$(cd "$NT_DIR/.." && pwd)"
WS="$(cd "$DIR/.." && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="$WS/devel/setup.bash"
ORANGE_SETUP="/home/orangepi/orange/devel/setup.bash"
MAP_ROOT="$HOME/test_data/mapping"

nt_help() {
  awk 'NR>2 && /^# =+$/{exit} NR>2{sub(/^# ?/, ""); print}' "$NT_SCRIPT"
  echo
  awk 'NR>2 && /^# =+$/{exit} NR>2{sub(/^# ?/, ""); print}' "${BASH_SOURCE[0]}"
}

log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }
ok()   { printf '[%s] [ OK ] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }
warn() { printf '[%s] [WARN] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }
err()  { printf '[%s] [FAIL] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG" >&2; }
hr()   { printf '%s\n' "----------------------------------------------------------------" | tee -a "$LOG"; }

# ---------------------------------------------------------------- 子进程 ----
# 同 reloc_test.sh：子进程各开一个会话（终端的 Ctrl+C 绕不到它们，由本脚本统一转发），
# pid 文件里记「PID + 启动时刻」，收尾时核对身份，PID 被复用也不会误停别的进程。
STARTED=" "
proc_starttime() {
  local _s; _s="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  _s="${_s##*)}"
  # shellcheck disable=SC2086
  set -- $_s; [ "$#" -ge 20 ] || return 1; printf '%s\n' "${20}"
}
child_pid() { # child_pid <pidfile>：核得上身份才输出 PID
  case "$STARTED" in *" $1 "*) ;; *) return 1 ;; esac
  local _pid _st; read -r _pid _st <"$1" 2>/dev/null || return 1
  [ -n "$_pid" ] && kill -0 "$_pid" 2>/dev/null || return 1
  [ "$(proc_starttime "$_pid")" = "$_st" ] || return 1
  printf '%s\n' "$_pid"
}
start_child() { # start_child <pidfile> <logfile> <命令...>
  local _pf="$1" _lf="$2"; shift 2
  STARTED="$STARTED$_pf "
  setsid --wait bash -c '
    _pf="$1"; shift; _cmd=("$@")
    _s="$(cat /proc/$$/stat 2>/dev/null)"; _s="${_s##*)}"; set -- $_s
    printf "%s %s\n" "$$" "${20:-}" >"$_pf"
    exec "${_cmd[@]}"
  ' _ "$_pf" "$@" 199>&- </dev/null >"$_lf" 2>&1 &
  local _i=0
  while [ "$_i" -lt 50 ] && [ ! -s "$_pf" ]; do sleep 0.1; _i=$((_i + 1)); done
}
# 后台子进程继承的 SIGINT 是「忽略」：fusion.sh 发 TERM（它对 TERM 装了收尾 trap，会把 bag 收尾、恢复对时）；
# roslaunch / rospy 节点自己注册了 SIGINT，发 INT。
stop_child() { # stop_child <pidfile> <名字> <INT|TERM> <最多等几秒>：每 5 s 补发一次；超时只提示，不强杀
  local _pid _i=0
  _pid="$(child_pid "$1")" || return 0
  log "停止$2（pid $_pid）"; kill -"$3" "$_pid" 2>/dev/null
  while child_pid "$1" >/dev/null; do
    sleep 0.5; _i=$((_i + 1))
    [ $((_i % 10)) -eq 0 ] && kill -"$3" "$_pid" 2>/dev/null
    if [ "$_i" -ge $(($4 * 2)) ]; then warn "$2 ${4} s 内没退出（pid $_pid），请手动处理"; return 1; fi
  done
}

fc_armed() { timeout 3 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q '^armed: True'; }

# ---------------------------------------------------------------- 收尾 ------
SHUTTING_DOWN=0
finish() { # finish <退出码> [force]：信号触发时飞机还解锁着就不收尾；启动失败、链路掉了用 force
  local _code="$1"
  [ "$SHUTTING_DOWN" -eq 1 ] && return
  if [ "${2:-}" != force ] && [ -n "${WAREHOUSE_TASK_ID:-}" ]; then
    local _fcu _ext
    _fcu="$(timeout 3 rostopic echo -n1 /mavros/state 2>/dev/null)"
    _ext="$(timeout 3 rostopic echo -n1 /mavros/extended_state 2>/dev/null)"
    if ! grep -qx 'connected: True' <<<"$_fcu" || ! grep -qx 'armed: False' <<<"$_fcu" \
        || grep -qE '^mode: .*OFFBOARD' <<<"$_fcu" || ! grep -qx 'landed_state: 1' <<<"$_ext"; then
      warn "收尾被拒：等待飞控确认已着地并上锁"
      return
    fi
  elif [ "${2:-}" != force ] && fc_armed; then
    echo; warn "飞机还在解锁状态，不收尾（停链路会断飞控的外部定位）。降落上锁后再 Ctrl+C"
    return
  fi
  SHUTTING_DOWN=1
  if [ -n "${WAREHOUSE_RUNNER_PID:-}" ]; then
    kill -INT -- "-$WAREHOUSE_RUNNER_PID" 2>/dev/null || true
    local _waited=0
    while kill -0 -- "-$WAREHOUSE_RUNNER_PID" 2>/dev/null && [ "$_waited" -lt 30 ]; do
      sleep 0.2
      _waited=$((_waited + 1))
    done
    if kill -0 -- "-$WAREHOUSE_RUNNER_PID" 2>/dev/null; then
      kill -TERM -- "-$WAREHOUSE_RUNNER_PID" 2>/dev/null || true
    fi
  fi
  hr; log "收尾（只停本次启动的进程，常驻 roscore / MAVROS 不动）"
  local _cleanup_ok=1
  if declare -F test_cleanup >/dev/null; then
    test_cleanup || _cleanup_ok=0
  fi
  stop_child "$PF_NAV" "导航" INT 20 || _cleanup_ok=0
  stop_child "$PF_CAR" "car_bridge" INT 10 || _cleanup_ok=0
  if (( NT_DLL )); then stop_child "$PF_DLL" "DLL 旁路" INT 15 || _cleanup_ok=0; fi
  stop_child "$PF_RELOC" "重定位节点" INT 10 || _cleanup_ok=0
  stop_child "$PF_FUSION" "定位链路 fusion.sh（录包收尾、恢复对时）" TERM 60 || _cleanup_ok=0
  rm -rf "$RUN_DIR"
  # 测完常直接断电：包的索引还在页缓存里就会丢
  log "写盘（sync）……"; sync
  hr
  local _b; for _b in "$OUT"/*.bag; do [ -f "$_b" ] && log "包：$_b（$(du -h "$_b" | cut -f1)）"; done
  for _b in "$OUT"/goal_*.json; do [ -f "$_b" ] && log "目标记录：$_b"; done
  log "全部记录：$OUT"
  if [ -n "${WAREHOUSE_TASK_ID:-}" ] && [ "$_cleanup_ok" -eq 1 ] && [ "$_code" -eq 130 ]; then
    ok "网页任务已收尾"
  fi
  if [ "${NAV_TEST_IN_TMUX:-0}" = 1 ]; then read -r -p "按回车关闭这个 tmux 会话 " _ </dev/tty; fi
  exit "$_code"
}

nt_check_chain() { # 每次发目标前确认链路还在
  child_pid "$PF_FUSION" >/dev/null || { err "定位链路 fusion.sh 已退出，看 $OUT/fusion.log"; return 1; }
  child_pid "$PF_RELOC" >/dev/null || { err "重定位节点已退出，看 $OUT/reloc.log"; return 1; }
  child_pid "$PF_NAV" >/dev/null || { err "导航已退出，看 $OUT/nav.log"; return 1; }
}

# These deployed task IDs keep their original recording and process footprint.
# New map-based tasks using nav_test_main inherit the optional DLL observer.
nt_default_dll() {
  case "$1" in
    test[1-9]|test10|test11|test_indoor[1-4]) return 1 ;;
    *) return 0 ;;
  esac
}

nav_test_main() {
  NT_SCRIPT="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
  MAP="${DEFAULT_MAP:-$MAP_ROOT/20261105_114605_outdoor}"
  RELOC_CONFIG="${DEFAULT_RELOC_CONFIG:-$WS/src/drone_relocalization/config/outdoor.yaml}"
  MAP_FRAME="${DEFAULT_MAP_FRAME:-$WS/src/car_bridge/config/map_frame_outdoor.yaml}"
  NAV_CONFIG="${DEFAULT_NAV_CONFIG:-$NT_DIR/outdoor_nav.yaml}"
  PREVIEW_RADIUS="${DEFAULT_PREVIEW_RADIUS:-0.2}"
  PREVIEW_HALF_HEIGHT="${DEFAULT_PREVIEW_HALF_HEIGHT:-0.15}"
  PREVIEW=0; LIVE_GRID_ONLY=0; EV_MODE="${DEFAULT_EV_MODE:-pose}"; FUSION_ARGS=()
  NT_DLL=0; NT_RAW_LIDAR=0
  if nt_default_dll "$TEST_NAME"; then NT_DLL=1; fi
  local _show_dll=0
  local _orig=("$@")
  while [ $# -gt 0 ]; do
    SHIFT=1
    case "$1" in
      --preview) PREVIEW=1 ;;
      --no-dll) NT_DLL=0 ;;
      --show-dll-config) _show_dll=1 ;;
      --record-raw-lidar) NT_RAW_LIDAR=1 ;;
      --live-grid-only) LIVE_GRID_ONLY=1 ;;
      --map) MAP="${2:-}"; SHIFT=2 ;;
      --reloc-config) RELOC_CONFIG="${2:-}"; SHIFT=2 ;;
      --map-frame) MAP_FRAME="${2:-}"; SHIFT=2 ;;
      --nav-config) NAV_CONFIG="${2:-}"; SHIFT=2 ;;
      --ev-mode) EV_MODE="${2:-}"; SHIFT=2 ;;
      --allow-unsynced-clock) FUSION_ARGS+=(--allow-unsynced-clock) ;;
      -h|--help) nt_help; exit 0 ;;
      *) if ! test_arg "$@"; then echo "未知参数: $1（-h 看用法）" >&2; exit 2; fi ;;
    esac
    [ "$#" -ge "$SHIFT" ] || { echo "$1 后面缺参数值" >&2; exit 2; }
    shift "$SHIFT"
  done
  case "$EV_MODE" in pose|odom) ;; *) echo "--ev-mode 只能是 pose 或 odom" >&2; exit 2 ;; esac

  if (( _show_dll )); then
    local _dll_text=关闭 _raw_text="沿用录包配置（默认关闭）"
    if (( NT_DLL )); then _dll_text="开启（只作对照）"; fi
    if (( NT_RAW_LIDAR )); then _raw_text="开启"; fi
    printf '%s\n' "任务：$TEST_NAME" "主定位先验图：$MAP" "DLL 旁路：$_dll_text" "原始雷达：$_raw_text"
    return 0
  fi

  # ssh 断开不能把测试停在半路（飞机可能还在天上）：不在 tmux 里就开一个会话再跑自己
  if [ -z "${TMUX:-}" ] && [ -t 0 ] && command -v tmux >/dev/null 2>&1; then
    if tmux has-session -t nav 2>/dev/null; then
      echo "已有 tmux 会话 nav（上一次的测试），接回去；要重新开始先在里面 Ctrl+C、按回车关掉它"
      exec tmux attach -t nav
    fi
    exec tmux new-session -s nav "NAV_TEST_IN_TMUX=1 $(printf '%q ' "$NT_SCRIPT" "${_orig[@]}")"
  fi

  MAP="$(cd "$MAP" 2>/dev/null && pwd)" || { echo "地图目录不存在" >&2; exit 2; }
  if declare -F test_preflight >/dev/null; then
    test_preflight || exit 2
  fi
  exec 199>"/tmp/mid360s-nav-test-$(id -u).lock"
  flock -n 199 || { echo "已有导航测试运行，请先收尾" >&2; exit 1; }
  OUT="$HOME/test_data/nav_test/$(date +%Y%m%d_%H%M%S)_$TEST_NAME"
  if [ -n "${WAREHOUSE_TASK_ID:-}" ]; then OUT="$OUT-${WAREHOUSE_TASK_ID:0:8}"; fi
  mkdir -p "$OUT"
  LOG="$OUT/nav_test.log"
  RUN_DIR="$(mktemp -d /tmp/nav_test.XXXXXX)"
  PF_FUSION="$RUN_DIR/fusion.pid"; PF_RELOC="$RUN_DIR/reloc.pid"; PF_CAR="$RUN_DIR/car.pid"; PF_NAV="$RUN_DIR/nav.pid"
  PF_DLL="$RUN_DIR/dll.pid"
  trap 'finish 130' INT
  if [ -n "${WAREHOUSE_TASK_ID:-}" ]; then
    trap 'finish 130' TERM
  else
    trap 'finish 143' TERM
  fi
  trap 'finish 129' HUP

  nt_bringup
  if (( NT_DLL )); then
    log "导航已就绪，异步启动 DLL 旁路（日志 $OUT/dll.log）"
    start_child "$PF_DLL" "$OUT/dll.log" nice -n 10 env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
      "$NT_DIR/start_dll_observer.sh" "$MAP" "$RELOC_CONFIG" "$OUT" || warn "DLL 旁路未启动"
  fi
  if [ -n "${WAREHOUSE_TASK_ID:-}" ]; then
    # The web supervisor signals only this shell. A separate job group lets
    # finish() check landed and disarmed before interrupting the flight runner.
    set -m
    (run_test) 199>&- </dev/null &
    WAREHOUSE_RUNNER_PID=$!
    set +m
    while kill -0 "$WAREHOUSE_RUNNER_PID" 2>/dev/null; do
      wait "$WAREHOUSE_RUNNER_PID"
    done
    unset WAREHOUSE_RUNNER_PID
  else
    run_test
  fi
  hr; log "测试动作结束。链路继续运行（飞控仍在收外部定位）：切回定点、降落、上锁后 Ctrl+C 收尾"
  local _nav_gone=0
  while :; do
    sleep 2
    child_pid "$PF_FUSION" >/dev/null || { err "定位链路 fusion.sh 已退出，看 $OUT/fusion.log"; finish 1 force; }
    if [ "$_nav_gone" -eq 0 ] && ! child_pid "$PF_NAV" >/dev/null; then warn "导航已退出，看 $OUT/nav.log"; _nav_gone=1; fi
  done
}

# ---------------------------------------------------------------- 启动 ------
nt_bringup() {
  hr; log "导航实飞 $TEST_NAME：$(test_desc)（记录：$OUT）"
  log "地图 $MAP"; log "重定位配置 $RELOC_CONFIG"; log "world map 标定 $MAP_FRAME"; log "导航配置 $NAV_CONFIG"
  log "外部定位送飞控：$([ "$EV_MODE" = odom ] && echo 'odom（位姿 + 速度）' || echo 'pose（只发位姿）')"
  [ "$PREVIEW" -eq 1 ] && warn "只预览：导航不发飞控（enable_output=false），膨胀 ${PREVIEW_RADIUS} / ${PREVIEW_HALF_HEIGHT} m"
  local _f
  for _f in "$ROS_SETUP" "$WS_SETUP" "$ORANGE_SETUP" "$MAP/map_world.pcd" "$MAP/meta.json" "$RELOC_CONFIG" "$MAP_FRAME" "$NAV_CONFIG" \
            "$DIR/fusion.sh" "$NT_DIR/nav_forward.py"; do
    [ -f "$_f" ] || { err "缺少 $_f"; finish 1 force; }
  done
  source "$ROS_SETUP"; source "$WS_SETUP"; source "$ORANGE_SETUP" --extend
  local _prior_file="" _prior_map_sha=""
  if [ "$LIVE_GRID_ONLY" -eq 0 ]; then
    _prior_file="$MAP/prior_grid.bin"
    if ! _prior_map_sha="$(python3 -B "$WS/src/drone_mapping/scripts/build_prior_grid" "$MAP" --verify 2>>"$LOG")"; then
      err "先验栅格校验失败，确认 $_prior_file 对应当前地图。详情见 $LOG"
      finish 1 force
    fi
    log "先验栅格 $_prior_file（地图 SHA-256 $_prior_map_sha）"
  else
    warn "本次仅使用实时栅格（--live-grid-only）"
  fi
  source "$DIR/ros_net.sh" || { err "ROS 核心地址不在（nmcli con up ros-core）"; finish 1 force; }
  timeout 5 rosnode list >/dev/null 2>&1 || { err "常驻 roscore 不可达（systemctl status mid360s-roscore）"; finish 1 force; }
  local _n _nodes; _nodes="$(timeout 5 rosnode list 2>/dev/null)"
  for _n in /drone_reloc /car_bridge /waypoint_manager /offboard_gateway; do
    printf '%s\n' "$_nodes" | grep -qx "$_n" && { err "已有 $_n 在跑（上一次没收尾？），先停掉"; finish 1 force; }
  done
  flock -n /tmp/mid360s_fusion.lock true || { err "fusion.sh / start_handheld_test.sh 已在运行，先停掉"; finish 1 force; }
  timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep -q '^connected: True' \
    || { err "常驻 MAVROS 没连上飞控（飞控没上电？）：导航要用飞控的位置"; finish 1 force; }
  ok "ROS master、MAVROS 和飞控都在"
  if [ "$EV_MODE" = odom ]; then
    # 协议不对 fusion.sh 自己会拒绝启动；飞控不融合速度只会白发，这里先查出来
    local _p; _p="$(timeout 5 rosparam get /mavros/fcu_protocol 2>/dev/null)"
    [ "$_p" = "v2.0" ] || { err "odom 方式要 MAVROS 用 MAVLink2，常驻 MAVROS 是 fcu_protocol=${_p:-读不到}"; finish 1 force; }
    # MAVROS 连上飞控后要等约 10 s 再拉参数表，拉完之前 mavparam get 读不到：刚上电就跑要多等一会
    local _i
    for _i in $(seq 7); do
      _p="$(timeout 10 rosrun mavros mavparam get EKF2_EV_CTRL 2>/dev/null | tail -1)"
      [ -n "$_p" ] && [ "$_p" -eq "$_p" ] 2>/dev/null && break
      _p=""
      [ "$_i" -eq 1 ] && log "MAVROS 还没拉完飞控参数表（刚上电要 20~30 s），等它再读 EKF2_EV_CTRL（最多 45 s）"
      [ "$_i" -lt 7 ] && sleep 5
    done
    if [ -z "$_p" ]; then
      if [ -n "${WAREHOUSE_TASK_ID:-}" ]; then
        err "读不到飞控 EKF2_EV_CTRL，无法确认外部速度融合，未启动导航"
        finish 1 force
      fi
      warn "读不到飞控 EKF2_EV_CTRL，没法确认飞控会融合外部速度"
    elif [ $((_p & 4)) -eq 0 ]; then
      err "飞控 EKF2_EV_CTRL=$_p 没有速度位（bit2），odom 方式发的速度不会被融合；改成带 bit2（如 15）或用 --ev-mode pose"
      finish 1 force
    else
      ok "飞控 EKF2_EV_CTRL=$_p，融合外部速度"
    fi
  fi
  if ! python3 - "$NAV_CONFIG" "$MAP" "$PREVIEW" 2>&1 <<'PY' | tee -a "$LOG"; then finish 1 force; fi
import os, sys, yaml
c = yaml.safe_load(open(sys.argv[1]))['config']
if c.get('map_id') != sys.argv[2]:
    sys.exit('导航配置 map_id %s 和地图 %s 不一致' % (c.get('map_id'), sys.argv[2]))
if sys.argv[3] == '0' and c.get('deployment_verified') is not True:
    sys.exit('导航配置 deployment_verified 不是 true，真实输出会被网关拒绝（只预览加 --preview）')
print('导航配置：三维限速 %.2f m/s、%.2f m/s²，竖直 %s，转向 %s°/s，目标范围 %s ~ %s（drone_odom）' % (c['max_vel'], c['max_acc'], '%.2f m/s、%.2f m/s²' % (c['max_vel_z'], c.get('max_acc_z', -1)) if 'max_vel_z' in c else '同水平', c.get('yaw_rate_deg', 10), c['bounds_min'], c['bounds_max']))
PY
  # EGO 的限速是 navigation.launch 的参数，不读这份配置：取出来传过去，EGO 和网关/执行器用同一组上限。
  # 竖直上限 max_vel_z / max_acc_z 可选，没写传 -1（和水平单轴上限相同）
  local _max_vel _max_acc _max_vel_z _max_acc_z
  read -r _max_vel _max_acc _max_vel_z _max_acc_z < <(python3 -c 'import sys, yaml; c = yaml.safe_load(open(sys.argv[1]))["config"]; print(c["max_vel"], c["max_acc"], c.get("max_vel_z", -1), c.get("max_acc_z", -1))' "$NAV_CONFIG")
  [ -n "$_max_acc_z" ] || { err "读不到导航配置的 max_vel / max_acc"; finish 1 force; }

  hr; log "步骤 1/4：载入地图（缓存不对就重建，室外图约 70 s，只第一次）"
  if ! python3 - "$MAP" "$RELOC_CONFIG" 2>&1 <<'PY' | tee -a "$LOG"; then err "地图载入失败"; finish 1 force; fi
import sys
from drone_relocalization.settings import load
from drone_relocalization.matching import MapModel
m = MapModel(sys.argv[1], load(sys.argv[2]))
print('地图 %d 点，载入 %.1f s，%s' % (len(m.P), m.load_s, '缓存命中' if m.cache_hit else '已重建缓存'))
PY

  hr; log "步骤 2/4：启动定位链路 fusion.sh（位姿送飞控，通道 6 录包到本次目录）。飞机放好别动"
  local _record_env=("NAV_TEST_DLL_TASK=")
  if (( NT_DLL )); then _record_env=("NAV_TEST_DLL_TASK=$TEST_NAME"); fi
  if (( NT_RAW_LIDAR )); then _record_env+=("RC_RECORD_RAW_CLOUD=true"); fi
  start_child "$PF_FUSION" "$OUT/fusion.log" env "${_record_env[@]}" "$DIR/fusion.sh" --no-camera --no-winch --bag-dir "$OUT" --bag-prefix "$TEST_NAME" --ev-mode "$EV_MODE" "${FUSION_ARGS[@]}"
  local _t=0
  until timeout 3 rostopic echo -n1 /fastlio/odom_world >/dev/null 2>&1; do
    child_pid "$PF_FUSION" >/dev/null || { err "fusion.sh 退出了，最后几行："; tail -n 8 "$OUT/fusion.log" | tee -a "$LOG"; finish 1 force; }
    _t=$((_t + 4)); [ "$_t" -ge 180 ] && { err "180 s 内没有 /fastlio/odom_world，看 $OUT/fusion.log"; finish 1 force; }
    sleep 1
  done
  ok "FAST-LIO 和重力桥已输出（/fastlio/odom_world）"

  if declare -F test_after_fusion >/dev/null && ! test_after_fusion; then
    err "定位启动后测试设备未就绪"; finish 1 force
  fi

  hr; log "步骤 3/4：重定位 + world map（第一个窗口算 6～12 s，第二个窗口确认 1～3 s）。继续别动"
  start_child "$PF_RELOC" "$OUT/reloc.log" rosrun drone_relocalization relocalize --map "$MAP" --config "$RELOC_CONFIG" --save "$OUT"
  start_child "$PF_CAR" "$OUT/car_bridge.log" roslaunch car_bridge car_bridge.launch drone_map:="$MAP" frames:="$MAP_FRAME"
  cat >"$RUN_DIR/wait_reloc.py" <<'PY'
import json, math, os, sys, time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
pid, logf = int(sys.argv[1]), open(sys.argv[2], 'a')
def out(s):
    line = '[%s] %s' % (time.strftime('%H:%M:%S'), s)
    print(line, flush=True); logf.write(line + '\n'); logf.flush()
st = dict(key=None, valid=False, state=None, pose=None)
def on_status(m):
    s = json.loads(m.data); st['valid'] = bool(s.get('valid')); st['state'] = s.get('state')
    key = (s.get('state'), s.get('why'))
    if key != st['key']:
        st['key'] = key
        out('重定位 %s：%s（已搜索 %d 次，确认 %d 次）' % (s.get('state'), s.get('why'), s.get('attempts', 0), s.get('confirmations', 0)))
def on_result(m):
    r = json.loads(m.data)
    if 'drone_map' not in r:
        if 'secs' in r: out('搜索结果：拒绝｜原因：%s' % r.get('why', ''))
        return
    p = r['drone_map']
    tag = '已确认' if r.get('confirmed') else ('可信候选，等下一个窗口确认' if r.get('candidate_ok') else '拒绝')
    out('搜索结果：%s｜位置 x=%.3f y=%.3f z=%.3f m（离原点水平 %.2f m）｜内点率 %.3f 残差 %.1f cm 歧义 %.2f｜算了 %.1f s%s' % (
        tag, p[0], p[1], p[2], math.hypot(p[0], p[1]), r.get('fitness', 0), 100 * r.get('rmse', 0), r.get('amb3d', 0),
        r.get('secs', 0), '' if r.get('candidate_ok') else '｜原因：' + r.get('why', '')))
def on_pose(m):
    st['pose'] = m
rospy.init_node('nav_test_wait_reloc', anonymous=True, disable_signals=True)
rospy.Subscriber('/drone_reloc/status', String, on_status, queue_size=10)
rospy.Subscriber('/drone_reloc/result', String, on_result, queue_size=10)
rospy.Subscriber('/drone_reloc/pose', PoseStamped, on_pose, queue_size=1)
t0 = time.monotonic()
try:
    while True:
        time.sleep(0.5)
        if st['valid'] and st['state'] == 'LOCALIZED' and st['pose'] is not None:
            p, q = st['pose'].pose.position, st['pose'].pose.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            out('已定位：drone_map x=%.3f y=%.3f z=%.3f m，航向 %.1f°（0° = 建图时机头），离原点水平 %.2f m' % (
                p.x, p.y, p.z, yaw, math.hypot(p.x, p.y)))
            sys.exit(0)
        try:
            os.kill(pid, 0)
        except OSError:
            out('重定位节点退出了（3 次都没确认、超时或输入断流）'); sys.exit(1)
        if time.monotonic() - t0 > 150:
            out('150 s 没定位成功'); sys.exit(2)
except KeyboardInterrupt:
    sys.exit(130)
PY
  local _rpid; _rpid="$(child_pid "$PF_RELOC")" || { err "重定位节点没起来，看 $OUT/reloc.log"; finish 1 force; }
  python3 "$RUN_DIR/wait_reloc.py" "$_rpid" "$LOG" || { err "重定位没成功，看 $OUT/reloc.log"; finish 1 force; }
  ok "重定位完成；world map 由 car_bridge 发（看 $OUT/car_bridge.log）"

  hr; log "步骤 4/4：启动导航（EGO + 执行器 + 网关，$([ "$PREVIEW" -eq 1 ] && echo 只预览 || echo 真实输出)）"
  local _nav_args=(config:="$NAV_CONFIG" enable_output:=true max_vel:="$_max_vel" max_acc:="$_max_acc" max_vel_z:="$_max_vel_z" max_acc_z:="$_max_acc_z")
  [ "$PREVIEW" -eq 1 ] && _nav_args=(config:="$NAV_CONFIG" enable_output:=false radius:="$PREVIEW_RADIUS" half_height:="$PREVIEW_HALF_HEIGHT" max_vel:="$_max_vel" max_acc:="$_max_acc" max_vel_z:="$_max_vel_z" max_acc_z:="$_max_acc_z")
  _nav_args+=(prior_grid_file:="$_prior_file" prior_map_sha256:="$_prior_map_sha")
  [ "$TEST_NAME" != test_indoor3_3 ] || _nav_args+=(allow_short_goals:=true)
  start_child "$PF_NAV" "$OUT/nav.log" roslaunch drone_navigation navigation.launch "${_nav_args[@]}"
  cat >"$RUN_DIR/wait_nav.py" <<'PY'
import json, os, sys, time
import rospy
from std_msgs.msg import Bool, String
pid, preview, require_prior = int(sys.argv[1]), sys.argv[2] == '1', sys.argv[3] == '1'
st = {}
def prior_ready(message):
    st['prior_ready'] = message.data
def on(m, name):
    try:
        st[name] = json.loads(m.data)
    except ValueError:
        pass
rospy.init_node('nav_test_wait_nav', anonymous=True, disable_signals=True)
if require_prior:
    rospy.Subscriber('/drone_nav/prior_grid_ready', Bool, prior_ready, queue_size=1)
for name in ('input_status', 'gateway_status'):
    rospy.Subscriber('/drone_nav/' + name, String, on, callback_args=name, queue_size=5)
t0 = time.monotonic()
try:
    while True:
        time.sleep(0.5)
        i, g = st.get('input_status', {}), st.get('gateway_status')
        if (i.get('state') == 'READY' and g is not None
                and (not require_prior or st.get('prior_ready') is True)):
            if g.get('real_output') is not (not preview):
                print('网关 real_output=%s，和要求的%s不一致（部署检查没过？）' % (g.get('real_output'), '只预览' if preview else '真实输出'), flush=True)
                sys.exit(3)
            sys.exit(0)
        try:
            os.kill(pid, 0)
        except OSError:
            sys.exit(1)
        if time.monotonic() - t0 > 60:
            print('60 s 没就绪：输入 %s，网关 %s，先验就绪 %s' % (i or '无消息', g or '无消息', st.get('prior_ready', '无消息') if require_prior else '跳过'), flush=True)
            sys.exit(2)
except KeyboardInterrupt:
    sys.exit(130)
PY
  local _npid; _npid="$(child_pid "$PF_NAV")" || { err "导航没起来，看 $OUT/nav.log"; finish 1 force; }
  if ! python3 "$RUN_DIR/wait_nav.py" "$_npid" "$PREVIEW" "$((1 - LIVE_GRID_ONLY))" 2>&1 | tee -a "$LOG"; then
    err "导航没就绪，最后几行："; tail -n 8 "$OUT/nav.log" | tee -a "$LOG"; finish 1 force
  fi
  ok "导航就绪（输入 READY，网关 $([ "$PREVIEW" -eq 1 ] && echo 只预览 || echo 真实输出)）"
}
