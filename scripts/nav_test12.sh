#!/usr/bin/env bash
# Outdoor test12: manual flight with existing localization and Channel 6 recording.
set -o pipefail

SELF=$(realpath -e -- "$0") || exit 1
source "$(dirname -- "$SELF")/nav_test/common.sh" || exit 1
MAP="$MAP_ROOT/20261105_114605_outdoor"
RELOC_CONFIG="$WS/src/drone_relocalization/config/outdoor.yaml"
DLL_MAP="${MAP}_screenshot_roi"
YWL_MAP="${MAP}_screenshot_roi"
RECORDER="$DIR/rc_record/record_outdoor_dll_ywl"
PROBE="$NT_DIR/test12_state.py"
TASK_ID=${WAREHOUSE_TASK_ID:-}
EV_MODE=pose
USE_DLL=1
USE_YWL=0
NO_DLL=0
DRY_RUN=0
args=("$@")

usage() {
  cat <<'HELP'
用法：nav_test12.sh [--ev-mode pose|odom] [--ywl|--no-dll] [--dry-run]
室外 test12：启动 FAST-LIO2、室外重定位、DLL 和 Channel 6 录包，人工遥控飞行。
--ywl 选择 YWL 地图定位，FAST-LIO2 继续给飞控供数。
--no-dll 按原手飞流程录包。
默认 pose。--dry-run 仅显示启动配置。
SSH 断开后用 tmux attach -t nav12 接回。
HELP
}

while (( $# > 0 )); do
  case "$1" in
    --ev-mode)
      if [[ $# -lt 2 || ! "$2" =~ ^(pose|odom)$ ]]; then usage >&2; exit 2; fi
      EV_MODE=$2
      shift
      ;;
    --ywl) USE_YWL=1 ;;
    --no-dll) NO_DLL=1 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done
if (( USE_YWL && NO_DLL )); then usage >&2; exit 2; fi
if (( USE_YWL || NO_DLL )); then USE_DLL=0; fi

if [[ -n "$TASK_ID" && ! "$TASK_ID" =~ ^[a-f0-9]{32}$ ]]; then
  echo '网页任务 ID 无效' >&2
  exit 2
fi

if (( DRY_RUN )); then
  printf '%s\n' "室外 test12 · 人工遥控" "地图：$MAP" "重定位配置：$RELOC_CONFIG" \
    "启动：Channel 6 录包待命 → fusion.sh --no-record --no-camera --no-winch --ev-mode $EV_MODE → 室外重定位" \
    "录包目录：$HOME/test_data/nav_test/<时间>_test12/"
  if (( USE_DLL )); then
    printf '%s\n' "DLL 裁图：$DLL_MAP（10 cm，1280 MiB）"
  elif (( USE_YWL )); then
    printf '%s\n' "YWL 裁图：$YWL_MAP" 'YWL 启动里程计就绪 → 室外重定位 → YWL TRACKING'
  else
    printf '%s\n' 'DLL：关闭'
  fi
  exit 0
fi

if [[ -z "${TMUX:-}" ]]; then
  [[ -z "$TASK_ID" ]] || { echo '网页 test12 需要在任务会话中启动' >&2; exit 1; }
  command -v tmux >/dev/null || { echo '缺少 tmux，不能启动 test12' >&2; exit 1; }
  if tmux has-session -t '=nav12' 2>/dev/null; then
    if [[ -t 0 ]]; then exec tmux attach-session -t '=nav12'; fi
    echo 'test12 已在运行：tmux attach -t nav12'
    exit 0
  fi
  flock -n "/tmp/mid360s-nav-test-$(id -u).lock" true || { echo '已有测试运行，请先收尾' >&2; exit 1; }
  if [[ -t 0 ]]; then exec tmux new-session -s nav12 -- "$SELF" "${args[@]}"; fi
  tmux new-session -d -s nav12 -- "$SELF" "${args[@]}" || exit 1
  echo 'test12 会话已创建：tmux attach -t nav12'
  exit 0
fi

for file in "$WS_SETUP" "$DIR/fusion.sh" "$DIR/ros_net.sh" "$PROBE" \
            "$MAP/map_world.pcd" "$MAP/meta.json" "$RELOC_CONFIG"; do
  [[ -r "$file" ]] || { echo "缺少 $file" >&2; exit 1; }
done
[[ -x "$RECORDER" ]] || { echo "找不到内部录包器：$RECORDER" >&2; exit 1; }
source "$ROS_SETUP" && source "$WS_SETUP" || exit 1
if (( USE_DLL )); then
  [[ -r "$ORANGE_SETUP" ]] || { echo '缺少 Orange 构建，请先部署 DLL 或使用 --no-dll' >&2; exit 1; }
  source "$ORANGE_SETUP" --extend || exit 1
  DLL_PACKAGE=$(rospack find fastlio_dll) || exit 1
  DLL_CONFIG="$DLL_PACKAGE/config/outdoor_large_field.yaml"
  for file in "$DLL_CONFIG" "$DLL_MAP/map_world.pcd" "$DLL_MAP/meta.json" "$DLL_MAP/manifest.json"; do
    [[ -r "$file" ]] || { echo "缺少 $file" >&2; exit 1; }
  done
elif (( USE_YWL )); then
  YWL_PACKAGE=$(rospack find ywl_localization) || { echo '缺少 YWL 构建，请先部署' >&2; exit 1; }
  YWL_CONFIG="$YWL_PACKAGE/config/mid360s.yaml"
  for file in "$YWL_CONFIG" "$YWL_MAP/map_world.pcd" "$YWL_MAP/meta.json" "$YWL_MAP/manifest.json"; do
    [[ -r "$file" ]] || { echo "缺少 $file" >&2; exit 1; }
  done
fi
source "$DIR/ros_net.sh" || exit 1

exec 199>"/tmp/mid360s-nav-test-$(id -u).lock"
flock -n 199 || { echo '已有测试运行，请先收尾' >&2; exit 1; }
nodes=$(timeout 5 rosnode list 2>/dev/null) || { echo '常驻 ROS master 不可达' >&2; exit 1; }
conflicts=$(printf '%s\n' "$nodes" | awk '/^\/(laserMapping|gravity_bridge|drone_reloc|offboard_gateway|waypoint_manager|rc_bag_recorder|fastlio_dll|ywl_localization|ywl_init_adapter)$/ || /^\/outdoor_dll_ywl_record_/')
if [[ -n "$conflicts" ]]; then
  printf '检测到已有定位、导航或录包节点，请先按原流程收尾：\n%s\n' "$conflicts" >&2
  exit 1
fi
flock -n /tmp/mid360s_fusion.lock true || { echo '已有 fusion 或雷达测试在运行' >&2; exit 1; }

check_ground() { timeout 8 python3 -B "$PROBE" ground; }
check_ground || exit 1
if [[ "$EV_MODE" == odom ]]; then
  protocol=$(timeout 5 rosparam get /mavros/fcu_protocol 2>/dev/null) || protocol=
  ev_ctrl=$(timeout 10 rosrun mavros mavparam get EKF2_EV_CTRL 2>/dev/null | tail -1) || ev_ctrl=
  if [[ "$protocol" != v2.0 || ! "$ev_ctrl" =~ ^[0-9]{1,3}$ ]] || (( (10#$ev_ctrl & 4) == 0 )); then
    echo 'odom 需要 MAVLink2 和 EKF2_EV_CTRL 的速度位，请检查或使用默认 pose' >&2
    exit 1
  fi
fi

OUT="$HOME/test_data/nav_test/$(date +%Y%m%d_%H%M%S)_test12"
[[ -z "$TASK_ID" ]] || OUT+="-${TASK_ID:0:8}"
mkdir -p -- "$(dirname "$OUT")" && mkdir -- "$OUT" || exit 1
LOG="$OUT/nav_test.log"
RUN_DIR=$(mktemp -d /tmp/nav_test12.XXXXXX) || exit 1
PF_FUSION="$RUN_DIR/fusion.pid"
PF_RELOC="$RUN_DIR/reloc.pid"
PF_RECORD="$RUN_DIR/record.pid"
PF_DLL="$RUN_DIR/dll.pid"
PF_YWL="$RUN_DIR/ywl.pid"
tail_pids=()
shutting_down=0

owns_localization() {
  child_pid "$PF_FUSION" >/dev/null || child_pid "$PF_RELOC" >/dev/null
}

finish12() {
  local code=$1
  (( shutting_down == 0 )) || return 1
  shutting_down=1
  if owns_localization && ! check_ground; then
    warn '收尾被拒：需要飞控已连接、已着地且已上锁。定位和录包继续运行'
    shutting_down=0
    return 1
  fi
  if ! stop_child "$PF_RECORD" 'Channel 6 录包' TERM 35; then
    shutting_down=0
    return 1
  fi
  # Flushing a raw bag can take seconds. Check the aircraft again before stopping localization.
  if owns_localization && ! check_ground; then
    warn '收尾被拒：等待飞控再次确认着地并上锁，定位继续运行'
    shutting_down=0
    return 1
  fi
  if ! stop_child "$PF_YWL" 'YWL 地图定位' INT 20 ||
     ! stop_child "$PF_DLL" 'DLL 地图定位' INT 20 ||
     ! stop_child "$PF_RELOC" '室外重定位' INT 12 ||
     ! stop_child "$PF_FUSION" 'fusion.sh' TERM 60; then
    shutting_down=0
    return 1
  fi
  local tail_pid bag
  for tail_pid in "${tail_pids[@]}"; do
    kill -TERM "$tail_pid" 2>/dev/null || true
    wait "$tail_pid" 2>/dev/null || true
  done
  for bag in "$OUT"/outdoor_dll_ywl_*.bag; do
    [[ ! -f "$bag" ]] || log "实飞对照包：$bag"
  done
  local clean=1
  if ! grep -qx 'TEST12_RECORD stopped' "$OUT/record.log" || compgen -G "$OUT/*.bag.active" >/dev/null; then
    warn "录包器未确认正常退出，请检查 $OUT/record.log"
    clean=0
  fi
  if ! sync -f "$OUT"; then
    warn "同步写盘失败，请检查 $OUT"
    clean=0
  fi
  log "test12 已收尾，日志和 bag：$OUT"
  rm -f -- "$PF_RECORD" "$PF_RELOC" "$PF_FUSION" "$PF_DLL" "$PF_YWL"
  rmdir -- "$RUN_DIR"
  if (( ! clean )); then exit 1; fi
  if [[ -n "$TASK_ID" && "$code" -eq 143 ]]; then
    ok '网页任务已收尾'
    exit 0
  fi
  exit "$code"
}
trap 'finish12 130' INT
trap 'finish12 143' TERM
trap 'finish12 129' HUP

wait_stage() {
  local stage=$1 seconds=$2
  shift 2
  local deadline=$((SECONDS + seconds)) pidfile
  while (( SECONDS < deadline )); do
    for pidfile in "$@"; do child_pid "$pidfile" >/dev/null || return 1; done
    if timeout 7 python3 -B "$PROBE" "$stage" "$MAP" >/dev/null 2>&1; then return 0; fi
    sleep 0.2
  done
  return 1
}

bringup12() {
  log "test12 · 手飞，外部定位 $EV_MODE，记录目录 $OUT"
  ok '飞控已连接、已着地且已上锁'
  sha256sum "$MAP/map_world.pcd" "$MAP/meta.json" "$RELOC_CONFIG" >"$OUT/map.sha256" || return 1
  local record_args=("$OUT")
  if (( USE_DLL )); then
    sha256sum "$DLL_MAP/map_world.pcd" "$DLL_MAP/meta.json" "$DLL_MAP/manifest.json" "$DLL_CONFIG" >>"$OUT/map.sha256" || return 1
    cp -- "$DLL_CONFIG" "$OUT/dll_config.yaml" && cp -- "$DLL_MAP/manifest.json" "$OUT/dll_map_manifest.json" || return 1
    record_args+=(--dll)
  elif (( USE_YWL )); then
    sha256sum "$YWL_MAP/map_world.pcd" "$YWL_MAP/meta.json" "$YWL_MAP/manifest.json" "$YWL_CONFIG" >>"$OUT/map.sha256" || return 1
    cp -- "$YWL_CONFIG" "$OUT/ywl_config.yaml" &&
      cp -- "$RELOC_CONFIG" "$OUT/reloc_config.yaml" &&
      cp -- "$YWL_MAP/manifest.json" "$OUT/ywl_map_manifest.json" || return 1
    { git -C "$WS" rev-parse HEAD && git -C "$WS" status --porcelain=v1 &&
      sha256sum "$WS/devel/lib/ywl_localization/ywl_localization_node"; } >"$OUT/ywl_version.txt" || return 1
    record_args+=(--ywl)
  fi
  start_child "$PF_RECORD" "$OUT/record.log" "$RECORDER" "${record_args[@]}"
  setsid tail -n +1 -F --pid="$$" "$OUT/record.log" &
  tail_pids+=("$!")
  wait_stage recorder 15 "$PF_RECORD" || { err '录包器未就绪，查看 record.log'; return 1; }
  ok 'Channel 6 录包已待命'
  local fusion_args=(--no-record --no-camera --no-winch --ev-mode "$EV_MODE")
  if "$DIR/fusion.sh" --help | grep -F -- '--timing-dir' >/dev/null; then
    fusion_args+=(--timing-dir "$OUT/timing")
  else
    warn '当前 fusion.sh 不支持单帧 timing/ 诊断'
  fi
  start_child "$PF_FUSION" "$OUT/fusion.log" "$DIR/fusion.sh" "${fusion_args[@]}"
  wait_stage fastlio 180 "$PF_RECORD" "$PF_FUSION" || { err 'FAST-LIO 未就绪，查看 fusion.log'; return 1; }
  ok 'FAST-LIO2 和重力桥已输出'
  if (( USE_YWL )); then
    log 'YWL 正在校验地图并建立固定索引'
    start_child "$PF_YWL" "$OUT/ywl.log" roslaunch ywl_localization ywl.launch \
      "map_dir:=$YWL_MAP" "legacy_map_dir:=$MAP" "reloc_config:=$RELOC_CONFIG" "calibration:=$YWL_CONFIG"
    setsid tail -n +1 -F --pid="$$" "$OUT/ywl.log" &
    tail_pids+=("$!")
    wait_stage ywl-bootstrap 150 "$PF_RECORD" "$PF_FUSION" "$PF_YWL" ||
      { err 'YWL 启动里程计未就绪，查看 ywl.log'; return 1; }
  fi
  local reloc_cmd=(rosrun drone_relocalization relocalize --map "$MAP" --config "$RELOC_CONFIG" --save "$OUT")
  if (( USE_YWL )); then
    # Keep the startup solver from competing with both estimators for BLAS workers.
    reloc_cmd=(env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 "${reloc_cmd[@]}")
  fi
  start_child "$PF_RELOC" "$OUT/reloc.log" "${reloc_cmd[@]}"
  if (( USE_DLL )); then
    log 'DLL 正在建立距离场并等待本次重定位确认'
    start_child "$PF_DLL" "$OUT/dll.log" roslaunch fastlio_dll observe.launch \
      "map_dir:=$DLL_MAP" "legacy_map_dir:=$MAP" "config:=$DLL_CONFIG"
    setsid tail -n +1 -F --pid="$$" "$OUT/dll.log" &
    tail_pids+=("$!")
  fi
  wait_stage reloc 150 "$PF_RECORD" "$PF_FUSION" "$PF_RELOC" || { err '室外重定位未确认，查看 reloc.log'; return 1; }
  local ready='室外重定位已确认'
  if (( USE_DLL )); then
    wait_stage dll 150 "$PF_RECORD" "$PF_FUSION" "$PF_RELOC" "$PF_DLL" ||
      { err 'DLL 未进入 TRACKING，查看 dll.log；可用 --no-dll 重新启动手飞录包'; return 1; }
    timeout 5 rosparam dump "$OUT/dll_params.yaml" /fastlio_dll || return 1
    ready+='，DLL TRACKING'
  elif (( USE_YWL )); then
    wait_stage ywl 15 "$PF_RECORD" "$PF_FUSION" "$PF_RELOC" "$PF_YWL" ||
      { err 'YWL 未进入 TRACKING，查看 ywl.log'; return 1; }
    timeout 5 rosparam dump "$OUT/ywl_params.yaml" /ywl_localization || return 1
    ready+='，YWL TRACKING'
  fi
  check_ground || return 1
  ok "$ready。拨 Channel 6 开包，保持静止录够 20 秒，再按原流程手飞"
  log '落地静止后拨回 Channel 6 封包。上锁后 Ctrl+C 收尾'
}

if ! bringup12; then
  err 'test12 启动未完成'
  finish12 1 || true
fi

reported=" "
while :; do
  sleep 2
  for stage in FUSION RELOC RECORD DLL YWL; do
    file_var="PF_$stage"
    pidfile="${!file_var}"
    if [[ -s "$pidfile" ]] && ! child_pid "$pidfile" >/dev/null && [[ "$reported" != *" $stage "* ]]; then
      warn "$stage 进程已退出，查看 $OUT 下的日志。其余组件保持运行"
      reported="$reported$stage "
    fi
  done
done
