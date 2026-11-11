#!/usr/bin/env bash
# ==============================================================================
# test8：和 test7 同一条全局路线，但每一段都先把机头转到前进方向再飞（前进 → 转 → 前进 → 转）。
#
# 用法：./nav_test8.sh [--delay 3] [--hold 0] [--point "x,y,z,yaw" ...] [公共选项]
#   --delay S   切 OFFBOARD 后多少秒开始发第一个目标（默认 3）
#   --hold S    每个"前进"段到点后保持多少秒（默认 0；转向段不保持）
#   --point P   自定义航点，可重复；只用位置，航向自动取相邻点连线方向
#   --check-segments / --max-start-offset M  见 nav_test/nav_global.py
#
# 默认路线（drone_map 绝对坐标，与 test7 相同；方括号里是 test8 自动算出的机头方向）：
#   (0,0,1)[起点] → (2,0,1)[0°] → (2,-2,1)[-90°] → (0,-2,1)[180°] → (0,0,1)[+90°]
# 实际节奏：前进 2 m → 原地转到下一段方向 → 前进 2 m → 原地转 → …（最后一点不再转）
#
# 航向的实现方式：每个拐点拆成两个目标——"前进"（下一拐点 + 该段方向）和"原地转向"（同一位置
# + 下一段方向）。原地转向走任务层"重复同一终点"的放行条件（位置 1 μm 内、误差 ≤0.15 m、
# 速度 ≤0.1 m/s），执行器按导航配置的 yaw_rate_deg 逐 tick 步进。
# 本脚本默认用 nav_test/outdoor_nav_yaw45.yaml（yaw_rate_deg=45，转 90° 约 2 s），其余配置与
# outdoor_nav.yaml 完全一致；test7 仍用 outdoor_nav.yaml 的 30°/s。
#
# 航点帧 drone_map = 室外 FLU，原点 = 建图/重定位启动时机体（雷达 IMU）所在位置，z=0 就是飞机
# 着地时的机体高度，不是地面海拔（z=1 约机体离地 1 m）。
#
# 开机后直接运行（飞控上电、遥控器开着），飞机放在重定位原点附近别动，等「导航就绪」：
#   1) 通道 6 拨到录包位
#   2) 人工定点起飞：先升到 0.70 m 以上（边界 z_min 0.40 + 余量 0.30，低于它会在发目标前被拒），
#      并把机头大致对着首段方向（默认路线是 map +x，即建图时的机头方向；任务层不接受首目标的
#      原地转向，偏了只会让第一段轻微侧飞），再把模式开关拨到 OFFBOARD
#   3) 飞完最后一点原地悬停；接管＝拨回定点或推杆，脚本只记录，不再发目标
#   4) 再测：切回定点再切 OFFBOARD 才重跑；不测了：人工降落、上锁后 Ctrl+C 收尾
# 不自动解锁、起飞或降落；不改限速、包络和边界。
# ==============================================================================
# common.sh 使用 TEST_NAME 和 test_arg 设置的 SHIFT。
# shellcheck disable=SC2034
TEST_NAME=test8
NAV_TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nav_test"
DEFAULT_NAV_CONFIG="$NAV_TEST_DIR/outdoor_nav_yaw45.yaml"
DELAY=3
HOLD=0
MAX_START=2
POINT_ARGS=()
CHECK_ARGS=()
test_desc() { echo "全局航点，每段先转向再前进（机头朝前进方向，转向 45°/s）"; }

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
    assert len(fields) == 4
    values = [float(x) for x in fields]
    assert all(math.isfinite(v) for v in values)
except (AssertionError, ValueError):
    sys.exit('--point 要 4 个数 "x,y,z,yaw_deg"，例如 "2,0,1,0"（test8 只用前三个）')
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

nav_turn() {
  python3 -B "$NT_DIR/nav_turn.py" "${POINT_ARGS[@]}" --hold "$HOLD" --delay "$DELAY" \
    --max-start-offset "$MAX_START" --out "$OUT" "${CHECK_ARGS[@]}" "$@" \
    > >(trap '' INT; exec tee -a "$LOG") 2>&1
}

run_test() {
  local _ans _rc
  hr
  [ -f "$NT_DIR/nav_turn.py" ] || { err "缺少 $NT_DIR/nav_turn.py"; return 1; }
  if [ "$PREVIEW" -eq 1 ]; then
    log "test8 预览：回车开始一轮，按目标移动/转向；转向段不保持，前进段保持 ${HOLD} s，不发飞控"
    while :; do
      _ans=""
      read -r -p "回车=预览一轮｜q=结束：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || return
      nav_turn --preview
      [ "$?" -eq 130 ] && return
    done
  fi
  log "test8：通道 6 录包 → 起飞到 0.70 m 以上、机头对首段 → 切 OFFBOARD，${DELAY} s 后按 drone_map 绝对坐标飞；每段先原地转向（45°/s）再前进"
  log "降落上锁后 Ctrl+C 收尾；原地转向是本项目第一次在实飞里使用，注意观察转向时的位置漂移"
  while :; do
    nt_check_chain || return
    nav_turn
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
