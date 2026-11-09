#!/usr/bin/env bash
# ==============================================================================
# 导航 test5：先升降 0.5 m，再斜飞往返，最后原地左转四次 90°。每段稳定到达后保持 5 s。
# 用法：./nav_test5.sh [--delay 3] [--ev-mode odom] [公共选项]
# 开机后先运行，飞机放在起飞点别动，等导航就绪；通道 6 录包。
# 人工定点起飞悬停，切 OFFBOARD，延时结束时冻结本轮起点、高度、航向。
# 一轮完成继续悬停；切回定点再切 OFFBOARD 才重新开始。故障/接管不续飞。
# 不测了：人工定点降落、上锁，Ctrl+C 收尾；不自动解锁、起飞或降落。
# --preview 不发飞控，回车执行一轮，需手持/模拟状态到点。
# test5 高度相对悬停 z0：z0+.5 -> z0 -> 前1m升.5 -> 返回 -> 原地左转90/180/270/360度。
# 每段先检查路径已观测/占据；原地转向承接已完成终点，10度/s，yaw误差5度内才到点。
# ==============================================================================
# common.sh 使用 TEST_NAME 和 test_arg 设置的 SHIFT。
# shellcheck disable=SC2034
TEST_NAME=test5
DELAY=3
test_desc() { echo "先升降 0.5 m，再斜飞往返，最后原地左转四次 90°"; }

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

nav_sequence() { python3 -B "$NT_DIR/nav_sequence.py" --test test5 --out "$OUT" --delay "$DELAY" "$@" > >(trap '' INT; exec tee -a "$LOG") 2>&1; }

run_test() {
  local _ans _rc
  hr
  [ -f "$NT_DIR/nav_sequence.py" ] || { err "缺少 $NT_DIR/nav_sequence.py"; return 1; }
  if [ "$PREVIEW" -eq 1 ]; then
    log "test5 预览：回车开始一轮，按目标移动/转向；每点保持 5 s，不发飞控"
    while :; do
      _ans=""
      read -r -p "回车=预览一轮｜q=结束：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || return
      nav_sequence --preview
      [ "$?" -eq 130 ] && return
    done
  fi
  log "test5：通道 6 录包 → 定点起飞悬停 → 切 OFFBOARD，${DELAY} s 后先升降 0.5 m，再斜飞往返，最后原地左转四次 90°，每点停 5 s"
  log "一圈后保持；切回定点再切 OFFBOARD 才重跑。降落上锁后 Ctrl+C 收尾"
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
