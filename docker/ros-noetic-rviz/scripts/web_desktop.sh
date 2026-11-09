#!/usr/bin/env bash
# All desktop services are local to this container. No ROS bridge is started.
set -euo pipefail
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
export XDG_RUNTIME_DIR="/tmp/rviz-runtime-$(id -u)"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
port="${RVIZ_WEB_PORT:-6080}"
geometry="${RVIZ_DESKTOP_SIZE:-1440x900}"
[[ "$port" =~ ^[0-9]+$ ]] && [ "$port" -ge 1024 ] && [ "$port" -le 55535 ] || exit 2
[[ "$geometry" =~ ^[0-9]+x[0-9]+$ ]] || exit 2
# Abstract X11 sockets share the host network namespace. A distinct display
# number per web port keeps parallel FAST-LIO / reloc / EGO desktops separate.
export DISPLAY=":$port"
pids=()
cleanup() {
  trap - EXIT INT TERM
  if [ "${#pids[@]}" -gt 0 ]; then
    kill -TERM "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
Xvfb "$DISPLAY" -screen 0 "${geometry}x24" -nolisten tcp & pids+=("$!")
ready=0
for _ in {1..50}; do
  if xdpyinfo >/dev/null 2>&1; then ready=1; break; fi
  kill -0 "${pids[0]}" 2>/dev/null || exit 1
  sleep .1
done
[ "$ready" -eq 1 ] || { echo 'Xvfb did not become ready' >&2; exit 1; }
openbox >/tmp/rviz-openbox.log 2>&1 & pids+=("$!")
# The image's noVNC client does not request lossy JPEG quality encodings.
# RFB and HTTP listen only on loopback, even with host networking.
x11vnc -display "$DISPLAY" -localhost -rfbport "$((port + 10000))" -nopw -forever -shared >/tmp/rviz-vnc.log 2>&1 & pids+=("$!")
websockify --web=/usr/share/novnc "127.0.0.1:$port" "127.0.0.1:$((port + 10000))" >/tmp/rviz-web.log 2>&1 & pids+=("$!")
"$@" & app_pid=$!; pids+=("$app_pid")
while kill -0 "$app_pid" 2>/dev/null; do
  for pid in "${pids[@]}"; do
    [ "$pid" != "$app_pid" ] || continue
    kill -0 "$pid" 2>/dev/null || { echo "Display process $pid exited" >&2; exit 1; }
  done
  sleep .5
done
set +e
wait "$app_pid"
status=$?
exit "$status"
