#!/usr/bin/env bash
# ==============================================================================
# test indoor 2：固定起始机头方向，前、右、后、左各 0.5 m，每点稳定到达后保持 2 s。
# 用法：./nav_test_indoor2.sh [--delay 3] [--ev-mode odom] [公共选项]
# 开机后先运行，飞机放在起飞点别动，等导航就绪；通道 6 录包。
# 人工定点起飞悬停，切 OFFBOARD，延时结束时冻结本轮起点、高度、航向。
# 四个目标按本轮起点一次算好，保持高度和航向；最后回到起点，继续悬停。
# 一圈结束后需切回定点再切 OFFBOARD 才重跑；启动已在 OFFBOARD 不触发。
# 倒计时切出取消；故障/接管不续飞，故障期间的模式切换不触发下一轮。
# 不测了：人工定点降落、上锁，Ctrl+C 收尾；不自动解锁、起飞或降落。
# --preview 不发飞控，回车执行一轮，需手持/模拟状态到点。
# 默认 20261021_110311_indoor + indoor.yaml + map_frame.yaml；实时点云检查整段路径。
# 室内预览沿用实飞膨胀 0.4 / 0.3 m，未知区域拒绝；起点/终点距边界至少 0.30 m。
# ==============================================================================
# common.sh 使用 TEST_NAME 和 test_arg 设置的 SHIFT。
# shellcheck disable=SC2034
TEST_NAME=test_indoor2
INDOOR_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_MAP="$HOME/test_data/mapping/20261021_110311_indoor"
DEFAULT_RELOC_CONFIG="$INDOOR_SCRIPT_DIR/../src/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$INDOOR_SCRIPT_DIR/../src/car_bridge/config/map_frame.yaml"
DEFAULT_NAV_CONFIG="$INDOOR_SCRIPT_DIR/nav_test/indoor_nav.yaml"
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3
DELAY=3
test_desc() { echo "0.5 m 正方形（前右后左），每点停 2 s，保持起始高度和航向"; }

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

nav_sequence() { python3 -B "$NT_DIR/nav_sequence.py" --test test_indoor2 --out "$OUT" --delay "$DELAY" "$@" > >(trap '' INT; exec tee -a "$LOG") 2>&1; }

run_test() {
  local _ans _rc
  hr
  [ -f "$NT_DIR/nav_sequence.py" ] || { err "缺少 $NT_DIR/nav_sequence.py"; return 1; }
  if [ "$PREVIEW" -eq 1 ]; then
    log "test indoor 2 预览：回车开始一圈，按四个目标移动；机头方向不变，每点保持 2 s，不发飞控"
    while :; do
      _ans=""
      read -r -p "回车=预览一轮｜q=结束：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || return
      nav_sequence --preview
      [ "$?" -eq 130 ] && return
    done
  fi
  log "test indoor 2：通道 6 录包 → 定点起飞悬停 → 切 OFFBOARD，${DELAY} s 后前、右、后、左各 0.5 m，机头方向不变，每点停 2 s"
  log "一圈后继续悬停；切回定点再切 OFFBOARD 才重跑。降落上锁后 Ctrl+C 收尾"
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
