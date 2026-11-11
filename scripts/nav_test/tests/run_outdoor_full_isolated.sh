#!/usr/bin/env bash
# Fresh loopback-only master + mocked FCU/navigation. No resident services or hardware.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/mid360s_ws/devel/setup.bash
source /home/orangepi/orange/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11443 ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$HOME/test_data/nav10_validation/$(date +%Y%m%d_%H%M%S)}"
python3 - <<'PY'
import socket
with socket.socket() as s:
    if s.connect_ex(('127.0.0.1',11443)) == 0:
        raise SystemExit('隔离端口11443已占用，拒绝连接已有Master')
PY
mkdir -p "$OUT"
export ROS_LOG_DIR="$OUT/roslogs"
PID=''
START=''
owned() { [ -n "$PID" ] && [ -r "/proc/$PID/stat" ] && [ "$(awk '{print $22}' "/proc/$PID/stat")" = "$START" ]; }
cleanup() {
  if owned; then
    kill -INT -- "-$PID" 2>/dev/null || true
    for _ in $(seq 50); do owned || break; sleep .1; done
    if owned; then kill -TERM -- "-$PID" 2>/dev/null || true; fi
    wait "$PID" 2>/dev/null || true
  fi
  echo "隔离验证记录：$OUT"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid roscore -p 11443 >"$OUT/master.log" 2>&1 & PID=$!
sleep .1
START="$(awk '{print $22}' "/proc/$PID/stat")"
ready=0
for _ in $(seq 50); do
  if rosparam list >/dev/null 2>&1; then ready=1; break; fi
  owned || { echo '隔离Master退出' >&2; exit 1; }
  sleep .2
done
[ "$ready" -eq 1 ] || { echo '隔离Master未就绪' >&2; exit 1; }
rosparam load "$DIR/../outdoor_nav_yaw45.yaml" /drone_nav
# Only mocks are started, no roslaunch navigation/fusion/MAVROS or arming service.
python3 -B -u "$DIR/integration_outdoor_full.py" --out "$OUT" 2>&1 | tee "$OUT/integration.log"
