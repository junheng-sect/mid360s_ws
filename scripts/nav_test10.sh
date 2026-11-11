#!/usr/bin/env bash
# ==============================================================================
# test10：人工解锁 → 自动起飞1 m → 六段综合路线 → OFFBOARD 慢速下降（0.1 m/s）→ 着地上锁。
# 只飞一圈；不自动解锁、不抢回模式、不因重新切 OFFBOARD 重跑。
# 用法：./nav_test10.sh [--delay 3] [--dry-run] [--preview] [公共选项]
#   --delay S    地面人工解锁后等待1~4 s（默认3）再切 OFFBOARD
#   --dry-run    仅打印路线/参数；不初始化ROS，不启动链路，不写运行记录
#   --preview    仅手持预览空中六段；不验证起降，不调用飞控切模式服务
# drone_map绝对点(x,y,z,yaw°)：
#   (2,0,1,-90) → (2,-2,1,-90) → (2,-2,2,0)
#   → (0,-2,2,0) → (0,-2,1,0) → (0,0,1,0)
# 到点后才转yaw；保持0/0/1/0/1/2 s。水平8 m，范围2×2 m，高度1–2 m。
# 配置outdoor_nav_yaw45.yaml；竖直0.3 m/s、yaw45°/s，降落0.1 m/s。解锁至上锁预计70–90 s。
# 飞机在原点附近上锁、机头朝map+x；等导航就绪后通道6录包、POSCTL地面人工解锁。
# 人员距航线至少4 m，检查头顶无树枝/电线。异常请用模式开关接管，不用Ctrl+C降落。
# 着地上锁后先拨回通道6停录包，再拿飞机，最后Ctrl+C收尾；常驻服务不动。
# ==============================================================================
set -o pipefail
TEST_NAME=test10
NAV_TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nav_test"
DEFAULT_NAV_CONFIG="$NAV_TEST_DIR/outdoor_nav_yaw45.yaml"
DEFAULT_PREVIEW_RADIUS=0.40
DEFAULT_PREVIEW_HALF_HEIGHT=0.30
DELAY=3
DRY_RUN=0
PREVIEW_ARG=()
SELECTED_NAV="$DEFAULT_NAV_CONFIG"
# Validate before common.sh bringup (including dry-run and all public arguments).
args=("$@")
while [ $# -gt 0 ]; do
  case "$1" in
    --delay)
      [ $# -ge 2 ] || { echo '--delay 后面缺参数' >&2; exit 2; }
      DELAY="$2"
      python3 - "$DELAY" <<'PY' || exit 2
import math, sys
try:
    v=float(sys.argv[1])
    assert math.isfinite(v) and 1 <= v <= 4
except (ValueError, AssertionError):
    sys.exit('--delay 要在1~4 s之间')
PY
      shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --preview) PREVIEW_ARG=(--preview); shift ;;
    --allow-unsynced-clock) shift ;;
    --map|--reloc-config|--map-frame|--nav-config|--ev-mode)
      [ $# -ge 2 ] && [ -n "$2" ] && [[ "$2" != --* ]] || { echo "$1 后面缺参数" >&2; exit 2; }
      [ "$1" != --nav-config ] || SELECTED_NAV="$2"
      if [ "$1" = --ev-mode ]; then
        case "$2" in pose|odom) ;; *) echo '--ev-mode 只能是pose或odom' >&2; exit 2 ;; esac
      fi
      shift 2 ;;
    -h|--help)
      sed -n '3,16s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"
      exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done
if [ "$DRY_RUN" -eq 1 ]; then
  exec python3 -B "$NAV_TEST_DIR/nav_outdoor_full.py" --dry-run --delay "$DELAY" --nav-config "$SELECTED_NAV" "${PREVIEW_ARG[@]}"
fi
test_desc() { echo 'test10：人工解锁后自动起降，水平/竖直/左右yaw，慢速0.1 m/s降落，仅一圈'; }
test_arg() {
  case "$1" in
    --delay) DELAY="$2"; SHIFT=2 ;;
    *) return 1 ;;
  esac
}
run_test() {
  nt_check_chain || return
  local extra=()
  [ "$PREVIEW" -eq 0 ] || extra+=(--preview)
  python3 -B "$NT_DIR/nav_outdoor_full.py" --delay "$DELAY" --out "$OUT" "${extra[@]}" \
    > >(trap '' INT; exec tee -a "$LOG") 2>&1
  local rc=$?
  if [ "$rc" -eq 0 ] && [ "$PREVIEW" -eq 0 ]; then
    ok 'test10完成且上锁：先停止通道6录包，再拿飞机，最后Ctrl+C收尾'
  else
    warn "test10动作退出（$rc），本次不重跑；链路保留，请飞手确认模式并安全落地上锁"
  fi
}
source "$NAV_TEST_DIR/common.sh"
nav_test_main "${args[@]}"
