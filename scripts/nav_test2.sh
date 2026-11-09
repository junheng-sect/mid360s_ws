#!/usr/bin/env bash
# ==============================================================================
# 导航实飞 test2：前、右、后、左各 1 m，四个角稳定到达后各保持 5 s，保持起始高度和航向。
#
# 路径: /home/orangepi/mid360s_ws/scripts/nav_test2.sh
# 用法: ./nav_test2.sh [--delay 3] [公共选项]
#   --delay S          切进 OFFBOARD 后原地保持多少秒再出发（默认 3，0 < S <= 30）
#
# 开机后运行，飞机放在起飞点别动，等「导航就绪」。之后不用碰键盘：
#   1) 通道 6 拨到录包位；定点模式解锁起飞，到约 1 m 悬停稳
#   2) 切 OFFBOARD，保持 3 s 后开始：前 1 m → 停 5 s → 右 1 m → 停 5 s
#      → 后 1 m → 停 5 s → 左 1 m 回起点 → 停 5 s，机头方向始终不主动改变
#   3) 一圈结束继续悬停；切回定点再切 OFFBOARD，从当时位置重新开始一圈
#   4) 中途接管或故障即终止本圈；重新切换模式才重新开始，不续飞剩余边
#   5) 不测了：切回定点降落，上锁后 Ctrl+C 收尾
# 到点判据沿用导航：位置 15 cm 内、速度 <=0.1 m/s、航向 5° 内持续 1 s，之后再保持 5 s。
# 四个目标由每圈开始时的位置和机头方向一次算好，保持该高度；实际到点有上述允许误差。
# --preview：不发飞控，回车开始一圈；需手持按提示移动或用模拟状态到点，q 结束。
# ==============================================================================
# common.sh 使用 TEST_NAME 和 test_arg 设置的 SHIFT。
# shellcheck disable=SC2034
TEST_NAME=test2
DELAY=3
test_desc() { echo "1 m 正方形（前右后左），每点停 5 s，保持起始高度和航向"; }

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

nav_square() { python3 -B "$NT_DIR/nav_square.py" --out "$OUT" --delay "$DELAY" "$@" > >(trap '' INT; exec tee -a "$LOG") 2>&1; }

run_test() {
  local _ans _rc
  hr
  [ -f "$NT_DIR/nav_square.py" ] || { err "缺少 $NT_DIR/nav_square.py"; return 1; }
  if [ "$PREVIEW" -eq 1 ]; then
    log "test2 预览：手持举高，回车开始一圈；按四个目标移动，到点后各保持 5 s；不发飞控"
    while :; do
      _ans=""
      read -r -p "回车=预览一圈正方形｜q=结束：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || return
      nav_square --preview
      [ "$?" -eq 130 ] && return
    done
  fi
  log "test2：通道 6 录包 → 定点起飞悬停 → 切 OFFBOARD，${DELAY} s 后绕 1 m 正方形，每点停 5 s"
  log "一圈后保持；切回定点再切 OFFBOARD 才重跑。降落上锁后 Ctrl+C 收尾"
  while :; do
    nt_check_chain || return
    nav_square
    _rc=$?
    if [ "$_rc" -eq 130 ]; then
      _ans=""
      read -r -p "自动发目标已停。回车=重新开始自动模式｜q=不再发（降落上锁后 Ctrl+C 收尾）：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; esac
    else
      warn "正方形测试退出（$_rc），本次不再自动发目标；降落上锁后 Ctrl+C 收尾"
      return
    fi
  done
}

source "$(dirname "${BASH_SOURCE[0]}")/nav_test/common.sh"
nav_test_main "$@"
