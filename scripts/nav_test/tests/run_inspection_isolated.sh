#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/mid360s_ws/devel/setup.bash
source /home/orangepi/orange/devel/setup.bash --extend
export ROS_MASTER_URI=http://127.0.0.1:11445 ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$(mktemp -d /tmp/test11-ros.XXXXXX)"
export ROS_LOG_DIR="$OUT/roslogs"
python3 - <<'PY'
import socket
with socket.socket() as sock:
    if sock.connect_ex(('127.0.0.1', 11445)) == 0:
        raise SystemExit('test11 isolated port is already in use')
PY
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
    echo "test11 isolated evidence: $OUT"
}
trap cleanup EXIT
setsid roscore -p 11445 >"$OUT/master.log" 2>&1 & pids+=("$!")
for _ in $(seq 50); do rosparam list >/dev/null 2>&1 && break; sleep .2; done
rosparam list >/dev/null
python3 - "$DIR/../outdoor_nav_inspection.yaml" "$OUT/config.yaml" <<'PY'
import sys, yaml
with open(sys.argv[1]) as stream:
    data = yaml.safe_load(stream)
data['config']['lever_body'] = [.1, 0., 0.]
with open(sys.argv[2], 'w') as stream:
    yaml.safe_dump(data, stream)
PY
setsid roslaunch drone_navigation navigation.launch config:="$OUT/config.yaml" enable_output:=false max_vel:=1.0 max_acc:=1.0 max_vel_z:=0.3 max_acc_z:=0.3 >"$OUT/nav.log" 2>&1 & pids+=("$!")
for _ in $(seq 50); do rosparam get /drone_nav/config/map_id >/dev/null 2>&1 && break; sleep .2; done
timeout 280s python3 -Bu "$DIR/integration_inspection.py" "$OUT" 2>&1 | tee "$OUT/integration.log"
