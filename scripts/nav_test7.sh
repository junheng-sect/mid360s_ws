#!/usr/bin/env bash
# ==============================================================================
# test7：按 drone_map 绝对（全局）坐标飞固定航点，每点稳定到达后保持 3 s。
#
# 用法：./nav_test7.sh [--delay 3] [--hold 3] [--point "x,y,z,yaw" ...] [公共选项]
#   --delay S   切 OFFBOARD 后多少秒开始发第一个点（默认 3）
#   --hold S    每个点到达后保持多少秒（默认 3）
#   --point P   自定义航点，可重复；给了就替换默认 5 点，按给出顺序飞
#   --check-segments / --max-start-offset M  见 nav_test/nav_global.py
#
# 默认航点（drone_map，m，第四个数是偏航角 °）：
#   (0,0,1,0) (2,0,1,0) (2,-2,1,0) (0,-2,1,0) (0,0,1,0)
#
# 航点帧 drone_map = 室外 FLU（x 建图时机头、y 左、z 上），原点 = 建图/重定位启动时机体
# （雷达 IMU）所在位置，也就是飞机停在地面时的机体高度，不是地面海拔：z=1 表示机体离地约 1 m。
# car_bridge 的 map 帧把地面定在 z=0，比它低 0.151 m（map_frame_outdoor.yaml，未实测）。
#
# 默认地图/重定位/标定/导航配置＝common.sh 的室外默认：
#   20261105_114605_outdoor + drone_relocalization/config/outdoor.yaml
#   + car_bridge/config/map_frame_outdoor.yaml + nav_test/outdoor_nav.yaml
#
# 开机后直接运行（飞控上电、遥控器开着），飞机放在重定位原点附近别动，等「导航就绪」：
#   1) 通道 6 拨到录包位
#   2) 人工定点起飞：先把飞机升到 0.70 m 以上（outdoor_nav.yaml 边界 z_min=0.40 加 0.30 m 余量，
#      低于它会在发目标前被边界预检拒绝），再把模式开关拨到 OFFBOARD，延时结束后按绝对坐标依次飞。
#      首个航点通常就在起飞点正下方，脚本会跳过发送、只保持 hold 秒（任务层拒绝 <0.2 m 的新目标）
#   3) 飞完最后一点保持后原地悬停；接管＝拨回定点或推杆，脚本只记录，不再发目标
#   4) 再测：切回定点再切 OFFBOARD 才重跑；不测了：人工降落、上锁后 Ctrl+C 收尾
# 不自动解锁、起飞或降落；不改导航限速、包络和边界（outdoor_nav.yaml）。
# ==============================================================================
# common.sh 使用 TEST_NAME 和 test_arg 设置的 SHIFT。
# shellcheck disable=SC2034
TEST_NAME=test7
DELAY=3
HOLD=3
MAX_START=2
POINT_ARGS=()
CHECK_ARGS=()
test_desc() { echo "固定全局航点（drone_map 绝对坐标），每点停 ${HOLD} s"; }

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
    sys.exit('--point 要 4 个数 "x,y,z,yaw_deg"，例如 "2,0,1,0"')
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
  python3 -B "$NT_DIR/nav_global.py" "${POINT_ARGS[@]}" --hold "$HOLD" --delay "$DELAY" \
    --max-start-offset "$MAX_START" --out "$OUT" "${CHECK_ARGS[@]}" "$@" \
    > >(trap '' INT; exec tee -a "$LOG") 2>&1
}

run_test() {
  local _ans _rc
  hr
  [ -f "$NT_DIR/nav_global.py" ] || { err "缺少 $NT_DIR/nav_global.py"; return 1; }
  if [ "$PREVIEW" -eq 1 ]; then
    log "test7 预览：回车开始一轮，按目标移动/转向；每点保持 ${HOLD} s，不发飞控"
    while :; do
      _ans=""
      read -r -p "回车=预览一轮｜q=结束：" _ans </dev/tty || return
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || return
      nav_global --preview
      [ "$?" -eq 130 ] && return
    done
  fi
  log "test7：通道 6 录包 → 起飞到 0.70 m 以上悬停 → 切 OFFBOARD，${DELAY} s 后按 drone_map 绝对坐标依次飞航点，每点停 ${HOLD} s"
  log "首点若已在起飞点正下方会跳过发送、只保持；降落上锁后 Ctrl+C 收尾"
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
