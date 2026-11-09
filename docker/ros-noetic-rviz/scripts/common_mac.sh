#!/usr/bin/env bash
# Mac-only network and Docker helpers. Ubuntu's common.sh remains unchanged.
CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RVIZ_SHARED_DIR="$CONFIG_DIR"
log()  { printf '[rviz-mac] %s\n' "$*" >&2; }
warn() { printf '[rviz-mac][提示] %s\n' "$*" >&2; }
die()  { printf '[rviz-mac][错误] %s\n' "$*" >&2; exit 1; }
RVIZ_HOST_OS="$(uname -s)"
[ "$RVIZ_HOST_OS" = Darwin ] || die "这是 Mac 专用入口；Ubuntu 请继续使用原来的 run_rviz.sh / ego-rviz。"
if ! command -v docker >/dev/null 2>&1 && [ -x /Applications/Docker.app/Contents/Resources/bin/docker ]; then
  export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"
fi
# Independent trusted local configuration, never read Ubuntu's local.env.
if [ -f "$CONFIG_DIR/local.mac.env" ]; then source "$CONFIG_DIR/local.mac.env"; fi
IMAGE_TAG="${RVIZ_IMAGE_TAG:-ros-noetic-rviz:noetic-arm64-web}"
CONTAINER_NAME="${RVIZ_CONTAINER_NAME:-rviz_mid360s}"
BOARD_CORE_IP=10.250.32.1
DEFAULT_ROS_MASTER="http://$BOARD_CORE_IP:11311"
pick_board_ip() {
  local candidate
  for candidate in 192.168.31.32 192.168.8.32; do
    ifconfig | awk -v prefix="${candidate%.*}." '$1 == "inet" && index($2, prefix) == 1 {found=1} END {exit !found}' \
      && { printf '%s' "$candidate"; return; }
  done
  python3 -c 'import socket; print(socket.gethostbyname("orangepi5max.local"))' 2>/dev/null || true
}
DEFAULT_BOARD_IP="${BOARD_IP:-$(pick_board_ip)}"
ensure_core_route() {
  local via="$1" current
  [ -n "$via" ] || return 1
  current="$(route -n get "$BOARD_CORE_IP" 2>/dev/null || true)"
  if printf '%s\n' "$current" | awk -v via="$via" '$1 == "gateway:" && $2 == via {ok=1} END {exit !ok}'; then return; fi
  log "添加经 $via 到 $BOARD_CORE_IP 的临时主机路由（需要 sudo）"
  if printf '%s\n' "$current" | awk '/flags:.*HOST/ {found=1} END {exit !found}'; then
    sudo route -n change -host "$BOARD_CORE_IP" "$via"
  else
    sudo route -n add -host "$BOARD_CORE_IP" "$via"
  fi
}
detect_ros_ip() {
  python3 - "$1" <<'PY'
import socket, sys
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.connect((sys.argv[1], 11311))
    print(sock.getsockname()[0])
PY
}
resolve_ros_master() { printf '%s' "${ROS_MASTER_URI:-$DEFAULT_ROS_MASTER}"; }
check_ros_master() {
  python3 - "$1" <<'PY'
import socket, sys, xmlrpc.client
socket.setdefaulttimeout(3)
try:
    code, _, pid = xmlrpc.client.ServerProxy(sys.argv[1], allow_none=True).getPid('/rviz_display_check')
    if code != 1:
        raise RuntimeError('getPid returned %s' % code)
except Exception as exc:
    sys.exit('ROS Master unreachable: %s' % exc)
print(pid)
PY
}
check_docker() {
  command -v docker >/dev/null 2>&1 || die "未找到 Docker Desktop。"
  docker info >/dev/null 2>&1 || die "Docker Desktop 引擎不可用，请先打开 Docker Desktop。"
}
check_image() {
  docker image inspect "$IMAGE_TAG" >/dev/null 2>&1 \
    || die "镜像 $IMAGE_TAG 不存在。先执行：${image_build_script:-$CONFIG_DIR/scripts/build_image_mac.sh}"
}
LAUNCH_FILE_IN_CONTAINER=/opt/rviz-ws/launch/display_gravity_aligned_mac.launch
LAUNCH_FILE_ON_HOST="$CONFIG_DIR/launch/display_gravity_aligned_mac.launch"
check_launch_file() { [ -f "$LAUNCH_FILE_ON_HOST" ] || die "缺少 launch 文件：$LAUNCH_FILE_ON_HOST"; }
source "$RVIZ_SHARED_DIR/scripts/display_runtime_mac.sh"
