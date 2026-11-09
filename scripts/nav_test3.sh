#!/usr/bin/env bash
# ==============================================================================
# 导航实飞 test3：切 OFFBOARD 原地悬停 3 s 后自动降落，直到落地上锁
#
# 路径: /home/orangepi/mid360s_ws/scripts/nav_test3.sh
# 用法: ./nav_test3.sh [--hover 3] [公共选项]
#   --hover S          切进 OFFBOARD 后原地悬停多少秒再降落（默认 3）
#
# 降落用 PX4 自己的降落模式：悬停时间到，脚本经 MAVROS 把模式切到 AUTO.LAND，PX4 在原地按 MPC_LAND_SPEED
# （11.5 起 0.4 m/s；没有测距仪，一直按这个速度降到地面）降下来，着地后 COM_DISARM_LAND（2 s）自动上锁；
# 脚本一直盯到上锁。悬停期间是导航网关在 OFFBOARD 里保持原地。上锁后 PX4 会回到开关所在的 OFFBOARD，再飞前先拨回定点。
# 不支持 --preview（只预览时网关不发飞控，切不进 OFFBOARD）。
#
# 开机后直接运行（飞控上电、遥控器开着）。飞机放在起飞点（建图原点 3 m 内）别动，等「导航就绪」。
# 之后全程不用碰键盘：
#   1) 通道 6 拨到录包位（bag 和飞控 ulog 一起录）
#   2) 定点模式解锁起飞，爬到约 1 m 悬停稳；人离飞机 1 m 以上，正下方别有东西
#   3) 遥控器切 OFFBOARD：原地悬停 3 s 后自动切降落，着地后自动上锁（3 s 内切出 OFFBOARD 就不降）
#   4) 再测：模式开关拨回定点，重新解锁起飞，再切 OFFBOARD
#   5) 不测了：上锁后通道 6 拨回，Ctrl+C 收尾
# 中途接管：降落中动摇杆即切回定点（COM_RC_OVERRIDE=3），或拨模式开关；脚本只记录，不再切模式。
# ==============================================================================
TEST_NAME=test3
HOVER=3
test_desc() { echo "悬停 ${HOVER} s 后降落，直到落地上锁"; }

test_arg() { # 本测试自己的选项：认得就设 SHIFT 并返回 0
  case "$1" in
    --hover) HOVER="${2:-}"; SHIFT=2
             awk -v d="$HOVER" 'BEGIN{exit !(d+0 > 0 && d+0 <= 30)}' || { echo "--hover 要在 0~30 s 之间（不含 0）" >&2; exit 2; } ;;
    *) return 1 ;;
  esac
}

# tee 忽略 SIGINT：Ctrl+C 只该让 nav_land 停下，不能先把 tee 杀掉让它写输出时断管
nav_land() { python3 "$NT_DIR/nav_land.py" --out "$OUT" "$@" > >(trap '' INT; exec tee -a "$LOG") 2>&1; }

run_test() {
  local _ans _rc
  hr
  log "test3：通道 6 录包 → 定点起飞到约 1 m 悬停 → 切 OFFBOARD，${HOVER} s 后自动切 AUTO.LAND，着地后自动上锁"
  log "再测：模式开关拨回定点，重新解锁起飞再切 OFFBOARD。不测了：上锁后通道 6 拨回、Ctrl+C"
  while :; do
    nt_check_chain || return
    nav_land --hover "$HOVER"
    _rc=$?
    # 走到这里：Ctrl+C 时飞机还解锁着（收尾被拒），或 nav_land 自己出错退出
    if [ "$_rc" -eq 130 ]; then
      _ans=""
      read -r -p "自动降落已停。回车=重新开始自动模式｜q=不再触发（上锁后 Ctrl+C 收尾）：" _ans </dev/tty
      case "$_ans" in q|Q) return ;; esac
    else
      warn "自动降落退出（$_rc），3 s 后重新开始；切进 OFFBOARD 前不会切任何模式"
      sleep 3
    fi
  done
}

for _a in "$@"; do
  [ "$_a" = --preview ] && { echo "test3 不支持 --preview：只预览时网关不发飞控，切不进 OFFBOARD，没法测降落" >&2; exit 2; }
done
source "$(dirname "${BASH_SOURCE[0]}")/nav_test/common.sh"
nav_test_main "$@"
