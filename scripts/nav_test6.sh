#!/usr/bin/env bash
# ==============================================================================
# 导航实飞 test6：地面解锁后自动起飞到 1 m 并悬停
#
# 路径: /home/orangepi/mid360s_ws/scripts/nav_test6.sh
# 用法: ./nav_test6.sh [--delay 3] [公共选项]
#   --delay S          解锁后多少秒起飞（默认 3，1~4：PX4 解锁后 10 s 不起飞会自动上锁）
#
# 起飞走导航网关的 OFFBOARD（PX4 的 AUTO.TAKEOFF 在外部航向下用不了，没有全局位置）：解锁后网关在地面预发送
# （设定值压在飞机下方，电机怠速），倒计时完脚本切 OFFBOARD，网关按导航配置的 max_vel_z / max_acc_z（室外 0.3 / 0.3）
# 从地面平滑爬升 takeoff_height（室外配置 1.0 m，相对起飞时飞控本地位置），到高后原地保持悬停。PX4 自己做起转和推力斜坡。
# 不支持 --preview（只预览时网关不发飞控，切不进 OFFBOARD）。
#
# 开机后直接运行（飞控上电、遥控器开着）。飞机放在起飞点（建图原点 3 m 内）别动，等「导航就绪」。
# 之后全程不用碰键盘：
#   1) 通道 6 拨到录包位（bag 和飞控 ulog 一起录）；人离飞机 1 m 以上，头顶 1.5 m 内别有东西
#   2) 模式开关在定点，地面解锁：3 s 后自动切 OFFBOARD 起飞，约 5 s 爬到 1 m 悬停
#      倒计时内上锁或拨模式开关即取消
#   3) 落地：模式开关拨到 OFFBOARD 再拨回定点（或直接推杆）接管，手动降落上锁
#   4) 再测：上锁后重新解锁；不测了：通道 6 拨回，Ctrl+C 收尾
# 中途接管：推杆即切回定点（COM_RC_OVERRIDE=3），或拨模式开关；脚本只记录，不再切模式。
# 模式开关一直在定点：脚本切的 OFFBOARD 不跟开关位置，所以拨回定点要先拨走再拨回来。
# ==============================================================================
TEST_NAME=test6
DELAY=3
test_desc() { echo "解锁 ${DELAY} s 后自动起飞到 takeoff_height 并悬停"; }

test_arg() { # 本测试自己的选项：认得就设 SHIFT 并返回 0
  case "$1" in
    --delay) DELAY="${2:-}"; SHIFT=2
             awk -v d="$DELAY" 'BEGIN{exit !(d+0 >= 1 && d+0 <= 4)}' || { echo "--delay 要在 1~4 s 之间" >&2; exit 2; } ;;
    *) return 1 ;;
  esac
}

# tee 忽略 SIGINT：Ctrl+C 只该让 nav_takeoff 停下，不能先把 tee 杀掉让它写输出时断管
nav_takeoff() { python3 "$NT_DIR/nav_takeoff.py" --out "$OUT" "$@" > >(trap '' INT; exec tee -a "$LOG") 2>&1; }

run_test() {
  local _ans _rc
  hr
  log "test6：通道 6 录包 → 定点模式在地面解锁 → ${DELAY} s 后自动切 OFFBOARD，爬升到 takeoff_height 悬停"
  log "落地：拨模式开关或推杆接管，手动降落上锁。不测了：上锁后通道 6 拨回、Ctrl+C"
  while :; do
    nt_check_chain || return
    nav_takeoff --delay "$DELAY"
    _rc=$?
    if [ "$_rc" -eq 130 ]; then
      _ans=""
      read -r -p "自动起飞已停。回车=重新开始｜q=不再触发（上锁后 Ctrl+C 收尾）：" _ans </dev/tty
      case "$_ans" in q|Q) return ;; esac
    else
      warn "自动起飞退出（$_rc），3 s 后重新开始；看到地面解锁前不会切任何模式"
      sleep 3
    fi
  done
}

for _a in "$@"; do
  [ "$_a" = --preview ] && { echo "test6 不支持 --preview：只预览时网关不发飞控，切不进 OFFBOARD，没法起飞" >&2; exit 2; }
done
source "$(dirname "${BASH_SOURCE[0]}")/nav_test/common.sh"
nav_test_main "$@"
