#!/usr/bin/env bash
# ==============================================================================
# test indoor 1：室内地图重定位后前进 1 m，保持起始高度和航向。到点保持 5 s 后继续悬停。
# 用法：./nav_test_indoor1.sh [--delay 3] [--ev-mode odom] [公共选项]
# 开机后先运行，飞机放在起飞点别动，等导航就绪；通道 6 录包。
# 人工定点起飞悬停，切 OFFBOARD，延时结束时冻结本轮起点、高度、航向。
# 每次重新切入 OFFBOARD 都从当时位置向前 1 m，不自动返回。启动已在 OFFBOARD 不触发。
# 倒计时切出取消；故障/接管不续飞，故障期间的模式切换不触发下一轮。
# 不测了：人工定点降落、上锁，Ctrl+C 收尾；不自动解锁、起飞或降落。
# --preview 不发飞控，回车执行一轮，需手持/模拟状态到点。
# 默认 20261021_110311_indoor + indoor.yaml + map_frame.yaml；实时点云检查整段路径。
# 室内预览沿用实飞膨胀 0.4 / 0.3 m，未知区域拒绝；起点/终点距边界至少 0.30 m。
# ==============================================================================
# common.sh 使用 TEST_NAME 和 test_arg 设置的 SHIFT。
# shellcheck disable=SC2034
TEST_NAME=test_indoor1
INDOOR_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_MAP="$HOME/test_data/mapping/20261021_110311_indoor"
DEFAULT_RELOC_CONFIG="$INDOOR_SCRIPT_DIR/../src/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$INDOOR_SCRIPT_DIR/../src/car_bridge/config/map_frame.yaml"
DEFAULT_NAV_CONFIG="$INDOOR_SCRIPT_DIR/nav_test/indoor_nav.yaml"
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3
DELAY=3
test_desc() { echo "前进 1 m，保持起始高度和航向"; }

test_arg() {
  case "$1" in
    --delay)
      DELAY="${2:-}"
      # shellcheck disable=SC2034
      SHIFT=2
      python3 - "$DELAY" <<'PY' || exit 2
import math, sys
try:
    delay = float(sys.argv[1])
    assert math.isfinite(delay) and 0 < delay <= 30
except (ValueError, AssertionError):
    sys.exit('--delay 要在 0~30 s 之间（不含 0）')
PY
      ;;
    *) return 1 ;;
  esac
}

nav_sequence() { python3 -B "$NT_DIR/nav_sequence.py" --test test_indoor1 --out "$OUT" --delay "$DELAY" "$@" > >(trap '' INT; exec tee -a "$LOG") 2>&1; }

run_test() {
  local _ans _rc
  hr
  [ -f "$NT_DIR/nav_sequence.py" ] || { err "缺少 $NT_DIR/nav_sequence.py"; return 1; }
  if [ "$PREVIEW" -eq 1 ]; then
    log "test indoor 1 预览：回车开始一轮，按目标移动/转向；每点保持 5 s，不发飞控"
    while :; do
      _ans=""
      read -r -p "回车=预览一轮｜q=结束：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || return
      nav_sequence --preview
      [ "$?" -eq 130 ] && return
    done
  fi
  log "test indoor 1：通道 6 录包 → 定点起飞悬停 → 切 OFFBOARD，${DELAY} s 后前进 1 m，保持起始高度和航向，每点停 5 s"
  log "到点后保持；切回定点再切 OFFBOARD，再沿当时机头向前 1 m。降落上锁后 Ctrl+C 收尾"
  while :; do
    nt_check_chain || return
    nav_sequence
    _rc=$?
    if [ "$_rc" -eq 130 ]; then
      _ans=""
      read -r -p "自动发目标已停。回车=重新开始自动模式｜q=不再发（降落上锁后 Ctrl+C 收尾）：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; esac
    else
      warn "导航测试退出（$_rc），本次不再自动发目标；降落上锁后 Ctrl+C 收尾"
      return
    fi
  done
}

source "$(dirname "${BASH_SOURCE[0]}")/nav_test/common.sh"
nav_test_main "$@"
