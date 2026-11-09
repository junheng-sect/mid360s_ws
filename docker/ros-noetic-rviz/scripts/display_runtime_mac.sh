#!/usr/bin/env bash
# Mac-only launch contract for mid360s_ws and Orange EGO (Bash 3.2).
# Callers set image/name/home, command and extra_mounts, then call run_display.
require_value() { [ "$#" -ge 2 ] && [ -n "$2" ] || die "$1 缺少参数"; }
validate_web_port() {
  [[ "$web_port" =~ ^[0-9]+$ ]] && [ "$web_port" -ge 1024 ] && [ "$web_port" -le 55535 ] \
    || die "--web-port 需要 1024 到 55535 的整数（另保留 port+10000 给本机 VNC）"
}
print_docker_command() { printf 'docker'; printf ' %q' "$@"; printf '\n'; }

resolve_display_network() {
  master_host="$(python3 - "$ros_master_uri" <<'PY'
import sys
from urllib.parse import urlparse
try:
    uri = urlparse(sys.argv[1])
    assert uri.scheme == 'http' and uri.hostname and uri.port
    assert not uri.username and not uri.password and uri.path in ('', '/')
except (ValueError, AssertionError):
    sys.exit('ROS Master URI 必须是 http://HOST:PORT')
print(uri.hostname)
PY
  )" || die "ROS Master 地址无效：$ros_master_uri"
  if [ "$dry_run" -eq 0 ] && [ "$master_host" = "$BOARD_CORE_IP" ]; then
    ensure_core_route "$DEFAULT_BOARD_IP" \
      || die "无法配置到 $BOARD_CORE_IP 的路由；板端地址：${DEFAULT_BOARD_IP:-未发现}。请连接板端所在网络，或设置 BOARD_IP。"
  fi
  local target="$master_host"
  if [ "$master_host" = "$BOARD_CORE_IP" ] && [ -n "$DEFAULT_BOARD_IP" ]; then target="$DEFAULT_BOARD_IP"; fi
  if [ -n "$ros_ip_override" ]; then ros_ip="$ros_ip_override"
  elif ! ros_ip="$(detect_ros_ip "$target" 2>/dev/null)"; then
    if [ "$dry_run" -eq 1 ]; then ros_ip='<自动识别的宿主机地址>'
    else die "无法识别到 $target 的本机源 IP，请检查网络或使用 --ros-ip"; fi
  fi
  log "ROS_MASTER_URI=$ros_master_uri ROS_IP=$ros_ip"
}

check_mac_docker() {
  python3 - <<'PY'
import os, plistlib, sys
path = '/Applications/Docker.app/Contents/Info.plist'
if os.path.isfile(path):
    with open(path, 'rb') as f:
        version = plistlib.load(f)['CFBundleShortVersionString']
    if tuple(int(x) for x in version.split('.')[:2]) < (4, 34):
        sys.exit('Docker Desktop 至少需要 4.34；当前 ' + version)
PY
  [ "$(docker image inspect --format '{{.Architecture}}' "$IMAGE_TAG")" = arm64 ] \
    || die "Mac 显示镜像必须为 arm64：${IMAGE_TAG}，请重新构建 Mac 镜像"
  python3 "$RVIZ_SHARED_DIR/tools/probe_host_network.py" "$IMAGE_TAG" "$ros_ip" \
    || die "Docker host 网络检查失败。请启用 Docker Desktop → Settings → Resources → Network → Enable host networking；此检查不连接板端。"
}

wait_web_desktop() {
  python3 - "$web_port" <<'PY'
import sys, time, urllib.request
import socket
url = 'http://127.0.0.1:%s/vnc.html' % sys.argv[1]
for _ in range(100):
    try:
        with socket.create_connection(('127.0.0.1', int(sys.argv[1]) + 10000), timeout=.5) as sock:
            if not sock.recv(12).startswith(b'RFB '):
                raise RuntimeError('VNC not ready')
        with urllib.request.urlopen(url, timeout=.5) as response:
            if response.status == 200:
                sys.exit(0)
    except Exception:
        time.sleep(.2)
sys.exit('noVNC 未就绪，请查看 docker logs')
PY
}

run_display() {
  validate_web_port
  resolve_display_network
  local is_web=0
  [ "$run_mode" != rviz ] || is_web=1
  docker_args=(run --init --name "$CONTAINER_NAME" --network host --platform linux/arm64
    --hostname "$(hostname)" --user "$(id -u):$(id -g)"
    -e ROS_MASTER_URI="$ros_master_uri" -e ROS_IP="$ros_ip"
    -e DISABLE_ROS1_EOL_WARNINGS=1 -e HOME=/home/ros
    -v "$display_home:/home/ros" -v "$display_logs:/logs"
    -w /home/ros)
  if [ "$detach" -eq 1 ] || [ "$is_web" -eq 1 ]; then docker_args+=(-d)
  elif [ -t 0 ] && [ -t 1 ]; then docker_args+=(--rm -it)
  else docker_args+=(--rm); fi
  docker_args+=("${extra_mounts[@]}")
  if [ "$is_web" -eq 1 ]; then
    docker_args+=(--shm-size=512m -e RVIZ_WEB_PORT="$web_port"
      -e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe)
    display_command=(/usr/local/bin/rviz-web-desktop "${display_command[@]}")
  fi
  # Exit before any Docker calls, mkdir, route changes or container removal.
  if [ "$dry_run" -eq 1 ]; then
    [ "$master_host" != "$BOARD_CORE_IP" ] || log "实际运行时检查到 $BOARD_CORE_IP 的主机路由；dry-run 不修改路由"
    print_docker_command "${docker_args[@]}" "$IMAGE_TAG" "${display_command[@]}"
    return
  fi
  check_docker
  check_image
  check_mac_docker
  if [ "$master_check" -eq 1 ]; then
    check_ros_master "$ros_master_uri" >/dev/null || die "ROS Master 不可达：$ros_master_uri"
  fi
  if [ "$is_web" -eq 1 ]; then
    python3 - "$web_port" <<'PY_PORT'
import socket, sys
for port in (int(sys.argv[1]), int(sys.argv[1]) + 10000):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', port))
            sock.listen(1)
        except OSError:
            sys.exit('本机端口 %d 已占用；先 --stop 或换 --web-port' % port)
PY_PORT
  fi
  mkdir -p "$display_home" "$display_logs"
  if [ "$run_mode" = rviz ] && [ -n "$personal_config" ] && [ ! -e "$personal_config" ]; then
    python3 "$RVIZ_SHARED_DIR/tools/initialize_layout.py" "$display_template" "$personal_config"
  fi
  if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    docker rm -f "$CONTAINER_NAME" >/dev/null
  fi
  log "启动 ${CONTAINER_NAME}（${IMAGE_TAG}）"
  if [ "$is_web" -eq 1 ]; then
    docker "${docker_args[@]}" "$IMAGE_TAG" "${display_command[@]}" >/dev/null
    if ! wait_web_desktop; then
      docker logs --tail 40 "$CONTAINER_NAME" >&2
      docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
      return 1
    fi
    local url="http://localhost:$web_port/vnc.html?autoconnect=1&resize=scale"
    log "浏览器显示：${url}；关闭 RViz 窗口或 --stop 会停止整个显示容器"
    if [ "${open_browser:-1}" -eq 1 ]; then open "$url"; fi
  else
    exec docker "${docker_args[@]}" "$IMAGE_TAG" "${display_command[@]}"
  fi
}

stop_display() {
  if [ "$dry_run" -eq 1 ]; then print_docker_command stop "$CONTAINER_NAME"; return; fi
  check_docker
  if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then docker stop "$CONTAINER_NAME"; fi
}
