#!/usr/bin/env bash
# Private playback master. No camera device nodes or MAVROS.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source "$HOME/orange/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11448 ROS_IP=127.0.0.1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
unset ROS_HOSTNAME
out="${2:?expected source flight and output directory}"
mkdir -p "$out"
export ROS_LOG_DIR="$out/roslogs"
pids=()
cleanup() { local p; for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done; for p in "${pids[@]}"; do wait "$p" || true; done; }
trap cleanup EXIT
python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",11448)); s.close()'
setsid roscore -p 11448 >"$out/master.log" 2>&1 & pids+=("$!")
for _ in $(seq 50); do rosparam list >/dev/null 2>&1 && break; sleep .2; done
rosparam list >/dev/null
setsid roslaunch ov13855_apriltag_vision ov13855_apriltag_vision.launch corners_only:=true >"$out/vision.log" 2>&1 & pids+=("$!")
setsid roslaunch barcode_reader_camera barcode_reader_camera.launch layout_file:="$HOME/orange/src/barcode_reader_camera/config/layouts/qr_top_tag_bottom.yaml" pose_topic:=/indoor5/target_tag_pose save_dir:="$out/qr" save_frames:=all >"$out/decoder.log" 2>&1 & pids+=("$!")
python3 -Bu "$(dirname "$0")/replay_indoor5_images.py" "$1" "$out" 2>&1 | tee "$out/replay.log"
