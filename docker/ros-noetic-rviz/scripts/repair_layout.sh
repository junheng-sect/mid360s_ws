#!/usr/bin/env bash
# 修复 / 预防 RViz 左侧面板错乱（黑块、面板错位、内容重影、滚动条错位）。
#
# Qt5/XCB 在 Docker 私有 IPC 下的重绘残影由显示/回放入口的 --ipc=host 修复。
# 本脚本仅保留额外的宽度保护；窄面板仍错乱时应先确认容器以 host IPC 重新创建。
# 默认 12 MiB 是保守预警预算，不是故障阈值，也不能据此判定花屏根因。
# 参考：https://github.com/ros-visualization/rviz/issues/1628
#       https://github.com/ros-visualization/rviz/issues/1842
#
# 本脚本：测量左侧面板尺寸 -> 超过预算就把分隔条拖窄 -> 用实测值校验。
# 预算默认 12 MiB，可用环境变量 RV_PANEL_BUDGET_MB 覆盖。
#
#   ./scripts/repair_layout.sh              # 检查并（必要时）自动收窄
#   ./scripts/repair_layout.sh --report     # 只看尺寸，不改动
#   ./scripts/repair_layout.sh --window ID  # 指定窗口（默认取最宽的 RViz 主窗口）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

usage() {
  cat <<'EOF'
用法: repair_layout.sh [选项]

  --report       只报告左侧面板尺寸与预算，不做改动
  --window ID    指定 RViz 窗口 id（默认自动查找，多窗口时取最宽的那个）
  -h, --help     显示本帮助

左侧面板出现黑块/错位时执行一次即可；run_rviz.sh 启动时也会自动做同样的检查。
EOF
}

report_only=0
target_wid=""
while [ $# -gt 0 ]; do
  case "$1" in
    --report) report_only=1; shift ;;
    --window) target_wid="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "未知参数：$1" ;;
  esac
done

[ -n "${DISPLAY:-}" ] || die "未设置 DISPLAY：请在图形桌面的终端里运行。"
for tool in xdotool xprop xwininfo python3; do
  command -v "$tool" >/dev/null 2>&1 \
    || die "未找到 $tool：sudo apt-get install -y xdotool x11-utils python3"
done

if [ -n "$target_wid" ]; then
  wid="$target_wid"
else
  wid="$(find_rviz_window || true)"
  [ -n "$wid" ] || die "未找到 RViz 主窗口：先用 ./scripts/run_rviz.sh 启动，或用 --window 指定 id。"
fi

read -r w h sx sy < <(read_layout "$wid") || true
if [ -z "${w:-}" ] || [ -z "${h:-}" ] || [ "$w" -le 0 ] || [ "$h" -le 0 ]; then
  die "无法测量窗口 $wid 的左侧面板尺寸（窗口是否还在？）。"
fi

log "窗口 $wid：$(xdotool getwindowname "$wid" 2>/dev/null || echo 无标题)"
log "左侧面板 ${w}x${h}px，单次重绘约 $(repaint_mib "$w" "$h") MiB（预算 ${RV_PANEL_BUDGET_MB} MiB）"
if [ "${sx:--1}" -ge 0 ]; then
  log "分隔条位置：相对窗口 (${sx}, ${sy})，收窄目标不超过 ${RV_PANEL_TARGET_W}px"
else
  warn "未定位到面板与 3D 视图之间的分隔条：面板可能被隐藏，或窗口结构异常。"
fi

if [ $(( w * h * 4 )) -le "$(panel_budget_bytes)" ]; then
  log "结论：在预算内，出现黑块/错位的风险低。"
  exit 0
fi

if [ "$report_only" -eq 1 ]; then
  warn "结论：超过预算，面板可能出现黑块/错位；去掉 --report 执行即可自动收窄。"
  exit 1
fi

if ensure_panel_fits "$wid"; then
  log "提示：面板宽度会随窗口大小变化，若再次出现错乱，重跑本脚本即可。"
else
  exit 1
fi
