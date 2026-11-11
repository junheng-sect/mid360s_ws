#!/usr/bin/env bash
# ==============================================================================
# test9：竖直矩形——x=2 平面上 2 m × 2 m，从 1 m 爬到 3 m 再降回。
#
# 用法：./nav_test9.sh [--delay 3] [--hold 3] [--point "x,y,z[,yaw]" ...] [公共选项]
#   --delay S   切 OFFBOARD 后多少秒开始发第一个目标（默认 3）
#   --hold S    每个点到达后保持多少秒（默认 3）
#   --point P   自定义航点，可重复；给了就替换默认 5 点。给 3 个数＝位置（航向保持进入本轮时的机头），
#               给 4 个数＝位置 + 目标航向（度）
#   --check-segments / --max-start-offset M  见 nav_test/nav_global.py
#
# 默认路线（drone_map 绝对坐标，m）：
#   (2,0,1) → (2,-2,1) → (2,-2,3) → (2,0,3) → (2,0,1)
#   即先在 1 m 高度沿 −y 飞 2 m，竖直爬 2 m 到 3 m，再沿 +y 飞 2 m，最后竖直降 2 m 回起点。
#   全程机头保持进入本轮时的朝向（不主动转向；要转向就写第 4 个数）。
#
# 航点帧 drone_map = 室外 FLU，原点 = 建图/重定位启动时机体（雷达 IMU）所在位置，z=0 是飞机着地时的
# 机体高度，不是地面海拔：z=3 表示机体离地约 3 m。
#
# 导航配置用 nav_test/outdoor_nav_z3.yaml：与 outdoor_nav.yaml 完全相同，只把 z 上界 2.5 抬到 3.5
# （配合 goal_boundary_margin 0.30，允许航点到 3.20 m）。用默认 outdoor_nav.yaml（上界 2.20）时
# z=3 的两个点会在发目标前被边界预检拒掉。竖直限速仍是 max_vel_z 0.3 m/s，2 m 爬升约 8 s。
#
# 开机后直接运行（飞控上电、遥控器开着），飞机放在重定位原点附近别动，等「导航就绪」：
#   1) 通道 6 拨到录包位
#   2) 人工定点起飞到 0.70 m 以上（边界 z_min 0.40 + 余量 0.30），把模式开关拨到 OFFBOARD
#   3) 首个航点 (2,0,1) 离起飞点约 2 m，脚本允许 --max-start-offset 3 m；飞完最后一点原地悬停
#   4) 接管＝拨回定点或推杆；再测＝切回定点再切 OFFBOARD；不测了＝人工降落上锁后 Ctrl+C
# 不自动解锁、起飞或降落。
# ==============================================================================
# common.sh 使用 TEST_NAME 和 test_arg 设置的 SHIFT。
# shellcheck disable=SC2034
TEST_NAME=test9
NAV_TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nav_test"
DEFAULT_NAV_CONFIG="$NAV_TEST_DIR/outdoor_nav_z3.yaml"
DELAY=3
HOLD=3
MAX_START=3
POINT_ARGS=()
CHECK_ARGS=()
DEFAULT_POINTS=(2,0,1 2,-2,1 2,-2,3 2,0,3 2,0,1)
test_desc() { echo "竖直矩形 (2,0,1)→(2,-2,1)→(2,-2,3)→(2,0,3)→(2,0,1)，每点停 ${HOLD} s，机头不转"; }

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
    --hold)
      HOLD="${2:-}"
      # shellcheck disable=SC2034
      SHIFT=2
      python3 - "$HOLD" <<'PY' || exit 2
import math, sys
try:
    hold = float(sys.argv[1])
    assert math.isfinite(hold) and 0 <= hold <= 60
except (ValueError, AssertionError):
    sys.exit('--hold 要在 0~60 s 之间')
PY
      ;;
    --point)
      POINT_ARGS+=(--point "${2:-}")
      # shellcheck disable=SC2034
      SHIFT=2
      python3 - "${2:-}" <<'PY' || exit 2
import math, sys
fields = sys.argv[1].split(',')
try:
    assert len(fields) in (3, 4)
    values = [float(x) for x in fields]
    assert all(math.isfinite(v) for v in values)
except (AssertionError, ValueError):
    sys.exit('--point 要 3 或 4 个数 "x,y,z" 或 "x,y,z,yaw_deg"，例如 "2,-2,3"')
PY
      ;;
    --max-start-offset)
      MAX_START="${2:-}"
      # shellcheck disable=SC2034
      SHIFT=2
      python3 - "$MAX_START" <<'PY' || exit 2
import math, sys
try:
    value = float(sys.argv[1])
    assert math.isfinite(value) and value > 0
except (ValueError, AssertionError):
    sys.exit('--max-start-offset 要是正数（m）')
PY
      ;;
    --check-segments)
      CHECK_ARGS=(--check-segments)
      ;;
    *) return 1 ;;
  esac
}

nav_global() {
  local _args=() _p
  if [ "${#POINT_ARGS[@]}" -gt 0 ]; then
    _args=("${POINT_ARGS[@]}")
  else
    for _p in "${DEFAULT_POINTS[@]}"; do _args+=(--point "$_p"); done
  fi
  python3 -B "$NT_DIR/nav_global.py" "${_args[@]}" --hold "$HOLD" --delay "$DELAY" \
    --max-start-offset "$MAX_START" --out "$OUT" --record-prefix "$TEST_NAME" "${CHECK_ARGS[@]}" "$@" \
    > >(trap '' INT; exec tee -a "$LOG") 2>&1
}

run_test() {
  local _ans _rc
  hr
  [ -f "$NT_DIR/nav_global.py" ] || { err "缺少 $NT_DIR/nav_global.py"; return 1; }
  if [ "$PREVIEW" -eq 1 ]; then
    log "test9 预览：回车开始一轮，按目标移动；每点保持 ${HOLD} s，不发飞控"
    while :; do
      _ans=""
      read -r -p "回车=预览一轮｜q=结束：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || return
      nav_global --preview
      [ "$?" -eq 130 ] && return
    done
  fi
  log "test9：通道 6 录包 → 起飞到 0.70 m 以上 → 切 OFFBOARD，${DELAY} s 后飞竖直矩形（z 1→3 m，竖直限速 0.3 m/s），每点停 ${HOLD} s"
  log "导航配置 ${NAV_CONFIG}（z 上界 3.5）；降落上锁后 Ctrl+C 收尾"
  while :; do
    nt_check_chain || return
    nav_global
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
