#!/usr/bin/env bash
# Private master, synthetic sensors; never starts resident services or FCU output.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11439 ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
T_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
T_OUT="${1:-$HOME/test_data/nav45_validation/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$T_OUT"
export ROS_LOG_DIR="$T_OUT/roslogs"
T_PIDS=()
cleanup() {
  for ((i=${#T_PIDS[@]}-1;i>=0;i--)); do kill -INT -- "-${T_PIDS[i]}" 2>/dev/null || true; done
  for p in "${T_PIDS[@]}"; do wait "$p" || true; done
  echo "Evidence: $T_OUT"
}
trap cleanup EXIT
python3 - <<'PY'
import socket
with socket.socket() as s:
    if s.connect_ex(('127.0.0.1',11439))==0:raise SystemExit('private master port 11439 already in use')
PY
python3 - "$T_OUT/config.yaml" <<'PY'
import sys,yaml
c=yaml.safe_load(open('/home/orangepi/mid360s_ws/scripts/nav_test/outdoor_nav_long.yaml'))
c['config']['map_id']='synthetic'
yaml.safe_dump(c,open(sys.argv[1],'w'))
PY
setsid roscore -p 11439 >"$T_OUT/master.log" 2>&1 & T_PIDS+=("$!")
for _ in $(seq 50); do rosparam list >/dev/null 2>&1 && break; sleep .2; done
rosparam load "$T_OUT/config.yaml" /drone_nav
setsid roslaunch drone_navigation ego.launch >"$T_OUT/ego.log" 2>&1 & T_PIDS+=("$!")
for node in waypoint_manager trajectory_executor offboard_gateway; do
  setsid rosrun drone_navigation "$node" _enable_output:=false >"$T_OUT/$node.log" 2>&1 & T_PIDS+=("$!")
done
python3 -B -u "$T_DIR/integration_sequence.py" --out "$T_OUT" 2>&1 | tee "$T_OUT/integration.log"
