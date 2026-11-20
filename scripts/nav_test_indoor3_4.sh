#!/usr/bin/env bash
# ==============================================================================
# 室内 test3_4 · 手飞 OV13855 录像。飞机静置，等室内重定位确认和 OV13855 就绪，拨通道 6 同时录 bag 和 OV13855 无损视频。
# 全程遥控。通道 6 回拨后保留 1 秒尾段，降落上锁后 Ctrl+C 或从网页收尾。
# ==============================================================================
TEST_NAME=test_indoor3_4
NT_MANUAL_FLIGHT=1
SELF_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SELF_DIR/nav_test/common.sh"
DEFAULT_MAP="$MAP_ROOT/20261114_101654_indoor_fly_raw_000001"
DEFAULT_RELOC_CONFIG="$WS/src/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$WS/src/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_NAV_CONFIG="$NT_DIR/indoor_fly_nav.yaml"
DEFAULT_EV_MODE=odom

test_desc() { echo "手飞 · 通道 6 同步录制定位 bag 和 OV13855 无损视频"; }
test_arg() { return 1; }

test_preflight() {
  [ "$EV_MODE" = odom ] && [ "$NAV_CONFIG" -ef "$DEFAULT_NAV_CONFIG" ] && [ "$MAP_FRAME" -ef "$DEFAULT_MAP_FRAME" ] || return 1
  python3 -B "$NT_DIR/nav_indoor3.py" --verify-map "$MAP" || return 1
  FUSION_ARGS+=(--no-record) # Start the regular recorder after the OV13855 stream is ready.
}

test_after_fusion() {
  local nodes topics
  nodes="$(timeout 5 rosnode list)" && topics="$(timeout 5 rostopic list)" || return 1
  if grep -Eq '^/(ov13855_cam|ov13855_full_cam|rc_bag_recorder|indoor3_4_video)$' <<<"$nodes" ||
     grep -Eq '^/(ov13855|ov13855_full)/image_raw$' <<<"$topics"; then
    err "已有相机或录制任务占用设备"; return 1
  fi
  printf '%s\n' ov13855 > "$OUT/video_cameras.txt" || return 1
  PF_CAM="$RUN_DIR/video_cameras.pid"; PF_VIDEO="$RUN_DIR/video.pid"; PF_RECORD="$RUN_DIR/record.pid"
  start_child "$PF_CAM" "$OUT/camera.log" roslaunch "$NT_DIR/indoor3_4_camera.launch"
  start_child "$PF_VIDEO" "$OUT/video.log" env ROSCONSOLE_STDOUT_LINE_BUFFERED=1 nice -n 5 \
    rosrun dual_camera_recorder dual_camera_recorder_node "_directory:=$OUT" _ov13855_only:=true
  timeout 25 rostopic echo -n1 --filter 'm.data' /indoor3_4_video/ready | grep -q True || {
    err "OV13855 录像预热失败：查看 $OUT/camera.log 和 $OUT/video.log"; return 1; }
  ok "OV13855 1280×960/15 fps 灰度原图就绪"
  local dll_task=""
  (( NT_DLL )) && dll_task="$TEST_NAME"
  start_child "$PF_RECORD" "$OUT/record.log" env ROS_PACKAGE_PATH="$DIR:$ROS_PACKAGE_PATH" NAV_TEST_DLL_TASK="$dll_task" RC_RECORD_RAW_CLOUD="$NT_RAW_LIDAR" \
    roslaunch "$DIR/launch/fusion_aux.launch" record:=true winch:=false bag_prefix:="$TEST_NAME" bag_dir:="$OUT"
  timeout 15 rostopic echo -n1 /rc_record/status | grep -q 'healthy=True' || { err "通道 6 录包器未就绪"; return 1; }
  ok "Channel 6 录包已待命，OV13855 录像跟随 bag"
}

run_test() {
  log "室内重定位已确认，手飞 OV13855 录像待命"
  local last="" state
  while :; do
    state="$(cat "$OUT/video_state.txt" 2>/dev/null)"
    child_pid "$PF_VIDEO" >/dev/null || state="录像器退出，请查看 video.log；定位继续运行"
    child_pid "$PF_RECORD" >/dev/null || state="录包器退出，请查看 record.log；定位继续运行"
    if [[ "$state" != "$last" ]]; then log "VIDEO34 $state"; last="$state"; fi
    child_pid "$PF_FUSION" >/dev/null && child_pid "$PF_RELOC" >/dev/null || return 1
    sleep 1
  done
}

test_cleanup() {
  local ok=0
  if [ -n "${PF_RECORD:-}" ]; then stop_child "$PF_RECORD" "Channel 6 录包" INT 25 || ok=1; fi
  # Wait for the OPEN/CLOSED subscriber to consume teardown before interrupting the encoder.
  if [ -n "${PF_VIDEO:-}" ]; then
    for _ in $(seq 30); do
      grep -q '录制中' "$OUT/video_state.txt" 2>/dev/null || break
      sleep .1
    done
    stop_child "$PF_VIDEO" "OV13855 无损视频封存" INT 20 || ok=1
  fi
  if [ -n "${PF_CAM:-}" ]; then stop_child "$PF_CAM" "OV13855 相机" INT 15 || ok=1; fi
  return "$ok"
}

test_after_cleanup() {
  python3 -B "$NT_DIR/index_video_segments.py" "$OUT"
}

nav_test_main "$@"
