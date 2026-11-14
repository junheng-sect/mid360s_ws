#!/usr/bin/env bash
# test11: ~34 m, 1–3 m, three 3 s inspection stops; test10 takeoff/landing.
# ./nav_test11.sh [--delay 3] [--dry-run] [--preview] [公共选项]
# --dry-run 仅打印地图、航点、巡检及起降安排。
# --preview 只输出导航预览，要求从地图原点上方约 1 m 开始。
set -o pipefail
TEST_NAME=test11
NAV_TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nav_test"
DEFAULT_NAV_CONFIG="$NAV_TEST_DIR/outdoor_nav_inspection.yaml"
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
      sed -n '2,5s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"
      exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done
if [ "$DRY_RUN" -eq 1 ]; then
  exec python3 -B "$NAV_TEST_DIR/nav_outdoor_inspection.py" --dry-run --delay "$DELAY" --nav-config "$SELECTED_NAV" "${PREVIEW_ARG[@]}"
fi
test_desc() { echo 'test11：人工解锁后自动起降，34 m 连续航段、随动航向和三次巡检'; }
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
  python3 -B "$NT_DIR/nav_outdoor_inspection.py" --delay "$DELAY" --out "$OUT" "${extra[@]}" \
    > >(trap '' INT; exec tee -a "$LOG") 2>&1
  local rc=$?
  if [ "$rc" -eq 0 ]; then
    if [ "$PREVIEW" -eq 0 ]; then
      ok 'test11完成且上锁：先停止通道6录包，再拿飞机，最后Ctrl+C收尾'
    else
      ok 'test11预览完成'
    fi
  else
    warn "test11动作退出（$rc），本次不重跑；链路保留：地面保持上锁，若在空中请飞手接管并安全落地"
  fi
}
# Refuse a mismatched profile before starting localization or navigation.
python3 -B "$NAV_TEST_DIR/nav_outdoor_inspection.py" --dry-run --delay "$DELAY" --nav-config "$SELECTED_NAV" >/dev/null || exit 2
source "$NAV_TEST_DIR/common.sh"
nav_test_main "${args[@]}"
