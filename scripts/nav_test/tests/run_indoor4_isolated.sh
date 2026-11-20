#!/usr/bin/env bash
# Indoor4: private localhost master, synthetic camera/flight, no resident services or FCU output.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
variant=4
port=11446
if [[ ${1:-} == --indoor5 ]]; then variant=5; port=11447; shift; fi
export ROS_MASTER_URI=http://127.0.0.1:$port ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1
unset ROS_HOSTNAME
INDOOR_TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INDOOR_EVIDENCE="${1:-$HOME/test_data/nav_indoor${variant}_validation/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$INDOOR_EVIDENCE"
export ROS_LOG_DIR="$INDOOR_EVIDENCE/roslogs"
INDOOR_PIDS=()
cleanup() {
  local i p
  for ((i=${#INDOOR_PIDS[@]}-1;i>=0;i--)); do kill -INT -- "-${INDOOR_PIDS[i]}" 2>/dev/null || true; done
  for p in "${INDOOR_PIDS[@]}"; do wait "$p" || true; done
  printf 'Evidence: %s\n' "$INDOOR_EVIDENCE"
}
trap cleanup EXIT
python3 - "$port" <<'PY'
import socket, sys
with socket.socket() as sock:
    sock.settimeout(.2)
    if sock.connect_ex(('127.0.0.1', int(sys.argv[1]))) == 0:
        raise SystemExit('private master port already in use')
PY
python3 - "$INDOOR_TEST_DIR/../indoor_fly_nav.yaml" "$INDOOR_EVIDENCE/config.yaml" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1]))
assert cfg['config']['allow_unknown_above'] is False
cfg['config'].update(map_id='synthetic', allow_short_goals=True)
with open(sys.argv[2], 'w') as output:
    yaml.safe_dump(cfg, output)
PY
setsid roscore -p "$port" >"$INDOOR_EVIDENCE/master.log" 2>&1 & INDOOR_PIDS+=("$!")
for _ in $(seq 50); do rosparam list >/dev/null 2>&1 && break; sleep .2; done
rosparam list >/dev/null
rosparam load "$INDOOR_EVIDENCE/config.yaml" /drone_nav
setsid roslaunch drone_navigation ego.launch radius:=0.4 half_height:=0.3 max_vel:=0.5 max_acc:=0.5 max_vel_z:=0.3 max_acc_z:=0.3 >"$INDOOR_EVIDENCE/ego.log" 2>&1 & INDOOR_PIDS+=("$!")
for node in waypoint_manager trajectory_executor offboard_gateway; do
  setsid rosrun drone_navigation "$node" _enable_output:=false >"$INDOOR_EVIDENCE/$node.log" 2>&1 & INDOOR_PIDS+=("$!")
done
python3 -B -u "$INDOOR_TEST_DIR/integration_indoor${variant}.py" --out "$INDOOR_EVIDENCE" "${@:2}" 2>&1 | tee "$INDOOR_EVIDENCE/integration.log"
