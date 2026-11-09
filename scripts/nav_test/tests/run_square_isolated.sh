#!/usr/bin/env bash
# 单独 master + 合成状态，绝不连接板端常驻 master，不发布飞控设定值。
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/mid360s_ws/devel/setup.bash
source /home/orangepi/orange/devel/setup.bash --extend
set -u
export ROS_MASTER_URI=http://127.0.0.1:11435
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export PYTHONDONTWRITEBYTECODE=1
SQUARE_TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SQUARE_SCRIPT="$SQUARE_TEST_DIR/../nav_square.py"
# 允许在同一临时目录内验证部署候选文件。
[ -f "$SQUARE_SCRIPT" ] || SQUARE_SCRIPT="$SQUARE_TEST_DIR/nav_square.py"
SQUARE_EVIDENCE="$(mktemp -d /tmp/square-ros-check.XXXXXX)"
export ROS_LOG_DIR="$SQUARE_EVIDENCE/roslogs"
SQUARE_MASTER_PID=''
cleanup() {
  if [ -n "$SQUARE_MASTER_PID" ]; then
    kill -INT -- "-$SQUARE_MASTER_PID" 2>/dev/null || true
    wait "$SQUARE_MASTER_PID" || true
  fi
  printf 'ROS evidence: %s\n' "$SQUARE_EVIDENCE"
}
trap cleanup EXIT
python3 - <<'PY'
import socket
with socket.socket() as sock:
    sock.settimeout(.2)
    if sock.connect_ex(('127.0.0.1', 11435)) == 0:
        raise SystemExit('isolated test port 11435 already in use; refusing to reuse')
PY
setsid roscore -p 11435 >"$SQUARE_EVIDENCE/master.log" 2>&1 &
SQUARE_MASTER_PID=$!
for _ in $(seq 40); do
  rosparam list >/dev/null 2>&1 && break
  sleep .2
done
rosparam list >/dev/null
python3 -B "$SQUARE_TEST_DIR/test_nav_square.py" 2>&1 | tee "$SQUARE_EVIDENCE/unit.log"
python3 -B -u "$SQUARE_TEST_DIR/integration_square.py" "$SQUARE_SCRIPT" 2>&1 | tee "$SQUARE_EVIDENCE/integration.log"
