#!/usr/bin/env bash
# 本机 Docker RViz 公共变量、网络检查和面板保护。

CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 本机文件采用 Bash 赋值语法，只加载当前检出的可信文件；不纳入 Git。
if [ -f "$CONFIG_DIR/local.env" ]; then
  # shellcheck source=/dev/null
  source "$CONFIG_DIR/local.env"
fi
IMAGE_TAG="${RVIZ_IMAGE_TAG:-${IMAGE_TAG:-ros-noetic-rviz:noetic}}"
CONTAINER_NAME="${RVIZ_CONTAINER_NAME:-${CONTAINER_NAME:-rviz_mid360s}}"
DEFAULT_ROS_MASTER="http://192.168.31.32:11311"
DEFAULT_BOARD_IP="192.168.31.32"
RVIZ_CONFIG_IN_CONTAINER="/opt/rviz-ws/rviz/fastlio_mid360s.rviz"
# 显示 launch 仅启动 RViz；world 静态 TF 来自板端。
LAUNCH_FILE_IN_CONTAINER="/opt/rviz-ws/launch/display_gravity_aligned.launch"
LAUNCH_FILE_ON_HOST="$CONFIG_DIR/launch/display_gravity_aligned.launch"

# 日志输出：全部走 stderr，避免与脚本返回值混在一起。
log()  { printf '[rviz] %s\n' "$*" >&2; }
warn() { printf '[rviz][提示] %s\n' "$*" >&2; }
die()  { printf '[rviz][错误] %s\n' "$*" >&2; exit 1; }

# ---------- Docker ----------
check_docker() {
  command -v docker >/dev/null 2>&1 || die "未找到 docker 命令。"
  docker info >/dev/null 2>&1 || die "Docker 守护进程不可用：请确认 docker 服务已启动，且当前用户在 docker 组。"
}

check_image() {
  docker image inspect "$IMAGE_TAG" >/dev/null 2>&1 \
    || die "镜像 $IMAGE_TAG 不存在。先执行：$CONFIG_DIR/scripts/build_image.sh"
}

# 显示模式依赖 launch 文件；缺文件时提前报错。
check_launch_file() {
  [ -f "$LAUNCH_FILE_ON_HOST" ] \
    || die "缺少 launch 文件：$LAUNCH_FILE_ON_HOST"
}

check_nvidia_runtime() {
  local runtimes
  runtimes="$(docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null)"
  case " $runtimes " in
    *" nvidia "*) return 0 ;;
    *) die "Docker 未注册 nvidia runtime。执行：sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker" ;;
  esac
}

# GPU 检查：宿主机驱动可见，容器内也能看到同一块 GPU。
check_gpu_host() {
  command -v nvidia-smi >/dev/null 2>&1 || die "宿主机未找到 nvidia-smi，无法确认 GPU 驱动状态。"
  if ! nvidia-smi -L >/dev/null 2>&1; then
    die "宿主机 nvidia-smi 执行失败，请先修复显卡驱动。"
  fi
  nvidia-smi -L | head -1
}

# ---------- 显示 ----------
check_display_env() {
  [ -n "${DISPLAY:-}" ] || die "未设置 DISPLAY：请在图形桌面的终端里运行，或先 export DISPLAY=:0。"
  local num="${DISPLAY#*:}"; num="${num%%.*}"
  [ -S "/tmp/.X11-unix/X${num}" ] \
    || die "找不到 X11 socket /tmp/.X11-unix/X${num}：确认桌面会话使用的是 X11 且 DISPLAY=${DISPLAY} 正确。"
  if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    warn "xdpyinfo 未能连接 ${DISPLAY}：可能缺少授权，脚本会继续尝试 Xauthority 与 xhost 两种方式。"
  fi
}

# 生成供容器使用的 Xauthority 文件（把 FamilyLocal 改写为 FamilyWild，
# 这样容器内部主机名与宿主机不同也能匹配凭据）。
make_xauth_file() {
  local out="$CONFIG_DIR/runtime/.docker.xauth" src
  mkdir -p "$(dirname "$out")"
  : >"$out"

  for src in "${XAUTHORITY:-}" "$HOME/.Xauthority" "/run/user/$(id -u)/gdm/Xauthority"; do
    [ -n "$src" ] && [ -f "$src" ] || continue
    if xauth -f "$src" nlist 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$out" nmerge - 2>/dev/null; then
      if [ -s "$out" ]; then
        log "已从 $src 生成 X 授权文件：$out"
        return 0
      fi
    fi
  done

  warn "未能生成 X 授权凭据，将依赖 xhost 授权（见下一步）。"
  return 1
}

ensure_xhost_grant() {
  local user="${USER:-$(id -un)}"
  if xhost 2>/dev/null | grep -qi "SI:localuser:${user}"; then
    log "X 桌面已存在本机用户授权（SI:localuser:${user}）。"
    return 0
  fi
  if xhost +SI:localuser:"$user" >/dev/null 2>&1; then
    log "已临时授权本机用户访问 X 桌面：xhost +SI:localuser:${user}（会话结束后失效）。"
    return 0
  fi
  warn "xhost 授权失败，若窗口无法打开，请在桌面终端手动执行：xhost +SI:localuser:${user}"
}

# ---------- RViz 窗口操作 ----------
# 窗口最大化后左侧面板偶发黑块/面板错位（见 README「RViz 窗口最大化后左侧面板错乱」），
# 这两个函数用于自动最大化与一键重排。
#
# 列出所有 RViz 主窗口 id：xdotool search --class rviz 会把 3D 视图、停靠面板等
# 子窗口一并列出，因此再用 WM_CLASS（rviz）、标题（含 RViz）和宽度过滤。
list_rviz_windows() {
  local w cls name wd
  for w in $(xdotool search --class rviz 2>/dev/null); do
    cls="$(xprop -id "$w" WM_CLASS 2>/dev/null || true)"
    case "$cls" in *'"rviz", "rviz"'*) ;; *) continue ;; esac
    name="$(xdotool getwindowname "$w" 2>/dev/null || true)"
    case "$name" in *RViz*) ;; *) continue ;; esac
    wd="$(xwininfo -id "$w" 2>/dev/null | awk -F: '/^  Width/{gsub(/ /,"",$2); print $2}')"
    [ -n "${wd:-}" ] || continue
    [ "$wd" -gt 500 ] || continue
    printf '%s\n' "$w"
  done
}

# 取要处理的 RViz 主窗口：优先当前活动窗口（如果它正是 RViz），否则取最宽的那个——
# 多开时窗口尺寸常常完全一样，只按宽度挑会变成「看运气」，优先用户正在用的那扇更符合预期。
find_rviz_window() {
  local active w wd best="" best_w=0
  active="$(xdotool getactivewindow 2>/dev/null || true)"
  while read -r w; do
    [ -n "$w" ] || continue
    if [ -n "$active" ] && [ "$w" = "$active" ]; then printf '%s' "$w"; return 0; fi
    wd="$(xwininfo -id "$w" 2>/dev/null | awk -F: '/^  Width/{gsub(/ /,"",$2); print $2}')"
    [ -n "${wd:-}" ] || continue
    if [ "$wd" -gt "$best_w" ]; then best_w="$wd"; best="$w"; fi
  done < <(list_rviz_windows)
  [ -n "$best" ] && printf '%s' "$best"
}

# 在 $1（空格分隔的窗口 id 列表，启动前的快照）之外找 RViz 主窗口，
# 即本次启动新开的那一个。找不到返回非 0。
find_new_rviz_window() {
  local before="$1" w
  while read -r w; do
    [ -n "$w" ] || continue
    case " $before " in *" $w "*) continue ;; esac
    printf '%s' "$w"
    return 0
  done < <(list_rviz_windows)
  return 1
}

window_is_maximized() {
  xprop -id "$1" _NET_WM_STATE 2>/dev/null | grep -q '_NET_WM_STATE_MAXIMIZED_HORZ'
}

# ---------- 左侧面板宽度控制 ----------
# 本机 Qt5/XCB 的停靠面板在 Docker 私有 IPC 下会发生滚动残影，700px 窄栏也可复现。
# 显示/回放入口通过 --ipc=host 修复；QT_X11_NO_MITSHM 不足以保证 Qt5 禁用共享内存。
# 下面的尺寸预算只是额外的保守布局保护，不能单凭宽×高×4 推断 X11 请求或花屏根因。
# 默认 12 MiB 预警预算，可用 RV_PANEL_BUDGET_MB 覆盖；不代表实际故障阈值。
RV_PANEL_BUDGET_MB="${RV_PANEL_BUDGET_MB:-12}"
# 默认左栏宽度，随配置文件一起保存（rviz/fastlio_mid360s.rviz 的 QMainWindow State）。
# 实测：只保留 Displays + Views 时面板最窄能到 476px；带 Selection/Tool Properties/Time
# 时下限高达 1410px——那几个面板的内容决定了最小宽度，所以收窄不动时要先看面板组成。
RV_PANEL_TARGET_W="${RV_PANEL_TARGET_W:-700}"

# 测量工具：按实际可见子窗口的相邻边界定位分隔条，不猜坐标。
GEOMETRY_TOOL="$CONFIG_DIR/tools/rviz_window_geometry.py"

# 读取一次窗口布局，输出 "面板宽 窗高 分隔条x 分隔条y"；分隔条定位失败时为 -1 -1。
read_layout() {  # $1=主窗口 id
  [ -f "$GEOMETRY_TOOL" ] || return 1
  python3 "$GEOMETRY_TOOL" "$1" 2>/dev/null
}

panel_width()  { read_layout "$1" 2>/dev/null | awk '{print $1}'; }
panel_height() { read_layout "$1" 2>/dev/null | awk '{print $2}'; }

# 单次重绘数据量估算（MiB，两位小数）。窗高取主窗口高度：面板最大化时铺满窗高，
# 用窗高是偏保守的上界，与 README 里标定的 12 MiB 预算口径一致。
repaint_mib() {  # $1=面板宽 $2=窗高
  awk -v w="$1" -v h="$2" 'BEGIN { printf "%.2f", w * h * 4 / 1048576 }'
}

panel_budget_bytes() { printf '%s' $((RV_PANEL_BUDGET_MB * 1024 * 1024)); }

# 收窄目标：默认窄面板宽度与「预算九成」两者取小，留 10% 余量。
target_panel_width() {  # $1=窗高
  local by_w
  by_w=$(( RV_PANEL_BUDGET_MB * 1024 * 1024 * 9 / 10 / (4 * $1) ))
  [ "$by_w" -lt 1 ] && by_w=1
  if [ "$by_w" -lt "$RV_PANEL_TARGET_W" ]; then printf '%s' "$by_w"; else printf '%s' "$RV_PANEL_TARGET_W"; fi
}

# 等布局稳定：连续两次测到的「面板宽 窗高」一致。RViz 恢复窗口状态是异步的，
# 太早测量会读到中间状态，进而做一次多余的拖动。
wait_layout_stable() {  # $1=主窗口 id  $2=超时秒数（默认 10）
  local wid="$1" deadline=$(( SECONDS + ${2:-10} )) prev="" cur=""
  while [ "$SECONDS" -lt "$deadline" ]; do
    cur="$(read_layout "$wid" 2>/dev/null | awk '{print $1, $2}')"
    if [ -n "$cur" ] && [ "$cur" = "$prev" ]; then return 0; fi
    prev="$cur"
    sleep 1
  done
  return 0
}

# 拖动分隔条把面板收到目标宽度。只用实测到的分隔条坐标；抓住手柄中心后手柄跟着鼠标走，
# 两者相差 (分隔条x − 面板宽)，落点要补上这个偏移。分几步移动是为了让 Qt 跟得上。
drag_splitter_to() {  # $1=主窗口 id  $2=分隔条x  $3=当前面板宽  $4=分隔条y  $5=目标面板宽
  local wid="$1" sx="$2" pw="$3" sy="$4" target="$5"
  local wx wy off mx delta step
  read -r wx wy < <(xwininfo -id "$wid" 2>/dev/null | awk -F: '
      /Absolute upper-left X/{gsub(/ /,"",$2); x=$2}
      /Absolute upper-left Y/{gsub(/ /,"",$2); y=$2}
      END{print x, y}')
  [ -n "${wx:-}" ] && [ -n "${wy:-}" ] || return 1
  # 合成的鼠标事件只会送到指针下最上层的窗口。多开 RViz 时窗口位置常常完全重合，
  # 目标窗口若被盖住，拖动就会落到别人身上——实测 xdotool windowraise 在这套桌面下
  # 不会真正改变叠放次序，必须用 windowactivate（会顺带切换焦点，这里可以接受）。
  xdotool windowactivate --sync "$wid" 2>/dev/null || true
  sleep 0.3
  off=$(( sx - pw ))
  mx=$(( target + off ))
  [ "$mx" -lt 0 ] && mx=0
  delta=$(( mx - sx ))
  xdotool mousemove $(( wx + sx )) $(( wy + sy )); sleep 0.4
  xdotool mousedown 1; sleep 0.3
  step=$(( delta / 8 ))
  if [ "$step" -ne 0 ]; then
    for _ in 1 2 3 4 5 6 7; do
      xdotool mousemove_relative -- "$step" 0; sleep 0.12
    done
  fi
  xdotool mousemove --sync $(( wx + mx )) $(( wy + sy )); sleep 0.15
  xdotool mouseup 1; sleep 0.5
}

# 面板太宽就收窄到预算内；返回 0 表示当前已在预算内（含刚调整完的情况）。
ensure_panel_fits() {  # $1=主窗口 id  $2=说明（可选，用于日志前缀）
  local wid="$1" note="${2:-}" w h sx sy bytes budget target attempt
  budget="$(panel_budget_bytes)"
  read -r w h sx sy < <(read_layout "$wid") || true
  if [ -z "${w:-}" ] || [ -z "${h:-}" ] || [ "$w" -le 0 ] || [ "$h" -le 0 ]; then
    warn "${note}无法测量窗口 $wid 的左侧面板尺寸，跳过宽度检查。"
    return 1
  fi
  bytes=$(( w * h * 4 ))
  if [ "$bytes" -le "$budget" ]; then
    log "${note}左侧面板 ${w}x${h}px，单次重绘约 $(repaint_mib "$w" "$h") MiB（预算 ${RV_PANEL_BUDGET_MB} MiB）：在预算内。"
    return 0
  fi
  warn "${note}左侧面板 ${w}x${h}px，单次重绘约 $(repaint_mib "$w" "$h") MiB，超过 ${RV_PANEL_BUDGET_MB} MiB 预算；"
  warn "${note}面板可能出现黑块/错位，正在自动收窄（目标不超过 ${RV_PANEL_TARGET_W}px）。"
  # 拖动后重新测量，按实测差值继续修正；每次调整都用最新的分隔条位置。
  for attempt in 1 2 3; do
    if [ "${sx:--1}" -lt 0 ]; then
      warn "${note}第 ${attempt} 次：未定位到面板与 3D 视图之间的分隔条，无法自动收窄。"
      break
    fi
    target="$(target_panel_width "$h")"
    drag_splitter_to "$wid" "$sx" "$w" "$sy" "$target" || true
    read -r w h sx sy < <(read_layout "$wid") || true
    [ -n "${w:-}" ] && [ -n "${h:-}" ] || break
    if [ $(( w * h * 4 )) -le "$budget" ]; then
      log "${note}已收窄到 ${w}x${h}px（约 $(repaint_mib "$w" "$h") MiB，目标 ${target}px）。"
      return 0
    fi
    [ "$attempt" -lt 3 ] && sleep 1
  done
  warn "${note}自动收窄后仍为 ${w}x${h}px（约 $(repaint_mib "$w" "$h") MiB，预算 ${RV_PANEL_BUDGET_MB} MiB），未达标。"
  warn "${note}请手动把左侧面板往左拖窄；若拖不动，说明已到 Qt 的最小宽度限制（实测面板越少下限越小）。"
  return 1
}

# 最大化 / 还原：优先走 EWMH（装了 wmctrl 时与桌面环境无关），否则退化为 GNOME 的 Super+↑/↓。
set_window_maximized() {  # $1=窗口 id  $2=add|remove
  local wid="$1" action="$2"
  if command -v wmctrl >/dev/null 2>&1; then
    if [ "$action" = add ]; then
      wmctrl -i -r "$wid" -b add,maximized_vert,maximized_horz
    else
      wmctrl -i -r "$wid" -b remove,maximized_vert,maximized_horz
    fi
    return 0
  fi
  xdotool windowactivate --sync "$wid" 2>/dev/null || xdotool windowraise "$wid" 2>/dev/null || true
  if [ "$action" = add ]; then
    xdotool key --clearmodifiers super+Up
  else
    xdotool key --clearmodifiers super+Down
  fi
}

# 后台看护本次新开的 RViz 窗口：等它出现，按需最大化，并在最大化前后各查一次左侧面板宽度。
# 全程只认「启动前不存在」的窗口 id，不碰用户之前已经打开的那扇窗。
# 从桌面显示工具迁入；保留原函数接口，在线入口固定关闭自动最大化。
spawn_rviz_window_watcher() {  # $1=启动前已存在的窗口 id 列表  $2=1/0 自动最大化  $3=窗口出现后等待秒数
  local before="$1" auto_maximize="$2" delay="$3"
  (
    set +e   # 看护进程不该因为某一步失败就整体中断
    local deadline=$((SECONDS + 90)) wid=""
    while [ "$SECONDS" -lt "$deadline" ]; do
      wid="$(find_new_rviz_window "$before")"
      [ -n "$wid" ] && break
      wid=""
      sleep 1
    done
    if [ -z "$wid" ]; then
      warn "未找到本次新开的 RViz 窗口，跳过窗口布局处理"
      exit 0
    fi
    # 等待 RViz 恢复保存的布局后再测量，避免在初始化的中间状态拖动面板。
    sleep "$delay"
    wait_layout_stable "$wid" 10
    # 配置文件里保存的布局本身就可能超预算；关闭最大化时仅执行这次检查。
    ensure_panel_fits "$wid" "启动布局：" || true
    if [ "$auto_maximize" -eq 1 ]; then
      set_window_maximized "$wid" add
      sleep 1
      if window_is_maximized "$wid"; then
        log "已自动最大化 RViz 窗口（--no-maximize 可关闭）"
      else
        warn "自动最大化未生效：请在窗口里手动最大化（Super+↑），或安装 wmctrl 后重试"
      fi
      sleep 1
      # 最大化会改变窗高，单次重绘面积随之变化，所以再查一次。
      wait_layout_stable "$wid" 10
      ensure_panel_fits "$wid" "最大化后：" || true
    fi
  ) &
}

# ---------- ROS 网络 ----------
# 读取到板端路由的源地址，即本机参与 ROS 通信时应声明的 IP。
detect_ros_ip() {
  local target="${1:-$DEFAULT_BOARD_IP}" src
  src="$(ip route get "$target" 2>/dev/null | awk '{for (i = 1; i <= NF; i++) if ($i == "src") {print $(i + 1); exit}}')"
  [ -n "$src" ] || return 1
  printf '%s' "$src"
}

# 通过 XML-RPC getPid 确认 ROS Master 真的在跑，而不是端口被别的进程占用。
# 注意：rosmaster 的 getPid 必须带 caller_id 参数，否则返回 -1。
check_ros_master() {
  local uri="$1"
  python3 - "$uri" <<'PY'
import sys
import xmlrpc.client

uri = sys.argv[1]
try:
    code, _, pid = xmlrpc.client.ServerProxy(uri, allow_none=True).getPid("/rviz_display_check")
except Exception as exc:  # noqa: BLE001 - 任何异常都视为 Master 不可用
    print("unreachable: %s" % exc, file=sys.stderr)
    sys.exit(1)
if code != 1:
    print("getPid 返回异常：code=%s" % code, file=sys.stderr)
    sys.exit(1)
print(pid)
PY
}

resolve_ros_master() {
  printf '%s' "${ROS_MASTER_URI:-$DEFAULT_ROS_MASTER}"
}
