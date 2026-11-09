#!/usr/bin/env bash
# Full bag, including takeoff. Private master and map-only planner; no FCU output.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11440 ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
T_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
T_BAG="${1:?bag required}";T_OUT="${2:?output directory required}"
mkdir -p "$T_OUT";export ROS_LOG_DIR="$T_OUT/roslogs"
T_PIDS=()
cleanup() {
  for ((i=${#T_PIDS[@]}-1;i>=0;i--));do kill -INT -- "-${T_PIDS[i]}" 2>/dev/null || true;done
  for p in "${T_PIDS[@]}";do wait "$p" || true;done
  echo "Evidence: $T_OUT"
}
trap cleanup EXIT
python3 - <<'PY'
import socket
with socket.socket() as s:
    if s.connect_ex(('127.0.0.1',11440))==0:raise SystemExit('private port 11440 already in use')
PY
setsid roscore -p 11440 >"$T_OUT/master.log" 2>&1 & T_PIDS+=("$!")
for _ in $(seq 50);do rosparam list >/dev/null 2>&1 && break;sleep .2;done
rosparam set /use_sim_time true
rosparam load /home/orangepi/mid360s_ws/scripts/nav_test/outdoor_nav.yaml /drone_nav
setsid roslaunch drone_navigation ego.launch >"$T_OUT/ego.log" 2>&1 & T_PIDS+=("$!")
setsid python3 -B -u "$T_DIR/probe_climb.py" "$T_OUT/probe.json" >"$T_OUT/probe.log" 2>&1 & T_PIDS+=("$!")
sleep 3
rosbag play -q --clock "$T_BAG" --topics /drone_nav/odom /drone_nav/sensor_odom /drone_nav/cloud /mavros/state >"$T_OUT/play.log" 2>&1
