#!/usr/bin/env bash
# ==============================================================================
# 导航实飞 test1：前进 1 m，保持高度和航向（EGO 第一次实飞；和前视对准的梯形速度对比用）
#
# 路径: /home/orangepi/mid360s_ws/scripts/nav_test1.sh
# 用法: ./nav_test1.sh [--dist 1.0] [--delay 3] [公共选项]
#   --dist M           每次沿机头水平方向飞多少米（默认 1.0）
#   --delay S          切进 OFFBOARD 后原地保持多少秒再自动出发（默认 3）
#
# 开机后直接运行（飞控上电、遥控器开着）。飞机放在起飞点（建图原点 3 m 内）别动，等「导航就绪」。
# 之后全程不用碰键盘：
#   1) 通道 6 拨到录包位（bag 和飞控 ulog 一起录）
#   2) 定点模式解锁起飞，爬到约 1 m 悬停稳；人离飞机 1 m 以上，别站在机头前方
#   3) 遥控器切 OFFBOARD：原地保持 3 s 后自动前进 1 m，到点后原地保持（3 s 内切出 OFFBOARD 就不发）
#   4) 再测：切回定点再切 OFFBOARD，3 s 后往回飞 1 m；之后一前一后交替（没到点就不换方向）
#   5) 不测了：切回定点降落，上锁后 Ctrl+C 收尾
# 中途接管：动摇杆即切回定点（COM_RC_OVERRIDE=3），或切出 OFFBOARD；网关随即停止输出。
# 先不飞、只看 EGO 规划的轨迹：加 --preview，手持举高（离身体远一点），回车发目标，飞机不会动。
# ==============================================================================
TEST_NAME=test1
DIST=1.0
DELAY=3
test_desc() { echo "前进 ${DIST} m，保持高度和航向"; }

test_arg() { # 本测试自己的选项：认得就设 SHIFT 并返回 0
  case "$1" in
    --dist) DIST="${2:-}"; SHIFT=2 ;;
    --delay) DELAY="${2:-}"; SHIFT=2
             awk -v d="$DELAY" 'BEGIN{exit !(d+0 > 0 && d+0 <= 30)}' || { echo "--delay 要在 0~30 s 之间（不含 0）" >&2; exit 2; } ;;
    *) return 1 ;;
  esac
}

# tee 忽略 SIGINT：Ctrl+C 只该让 nav_forward 取消任务，不能先把 tee 杀掉让它写输出时断管
nav_forward() { python3 "$NT_DIR/nav_forward.py" --out "$OUT" "$@" > >(trap '' INT; exec tee -a "$LOG") 2>&1; }

run_test() {
  local _ans _rc
  hr
  if [ "$PREVIEW" -eq 1 ]; then
    log "test1 预览：手持举高后回车发目标（前进 ${DIST} m），飞机不会动；q 结束"
    while :; do
      _ans=""
      read -r -p "回车=前进 ${DIST} m｜q=结束：" _ans </dev/tty || continue
      case "$_ans" in q|Q) return ;; "") ;; *) continue ;; esac
      nt_check_chain || continue
      nav_forward --dist "$DIST" --yes
    done
  fi
  log "test1：通道 6 录包 → 定点起飞到约 1 m 悬停 → 切 OFFBOARD，${DELAY} s 后自动前进 ${DIST} m"
  log "再测：切回定点再切 OFFBOARD（一前一后交替）。不测了：切回定点降落，上锁后 Ctrl+C"
  while :; do
    nt_check_chain || return
    nav_forward --dist "$DIST" --auto-offboard "$DELAY"
    _rc=$?
    # 走到这里：Ctrl+C 时飞机还解锁着（收尾被拒），或 nav_forward 自己出错退出
    if [ "$_rc" -eq 130 ]; then
      _ans=""
      read -r -p "自动发目标已停。回车=重新开始自动模式｜q=不再发（降落上锁后 Ctrl+C 收尾）：" _ans </dev/tty
      case "$_ans" in q|Q) return ;; esac
    else
      warn "自动发目标退出（$_rc），3 s 后重新开始；切进 OFFBOARD 前不会发任何目标"
      sleep 3
    fi
  done
}

source "$(dirname "${BASH_SOURCE[0]}")/nav_test/common.sh"
nav_test_main "$@"
