# Shared camera and navigation setup for indoor AprilTag trials.
SURVEY_VARIANT="${TEST_NAME#test_}"
SURVEY_FLAGS=()
CAMERA_ARGS=()
SURVEY_RUNNER=nav_indoor3_1.py
SURVEY_LAUNCH="${SURVEY_VARIANT}_camera.launch"
if [[ "$SURVEY_VARIANT" == indoor3_3 ]]; then
  CAMERA_ARGS+=(upper_detection:=false)
  SURVEY_RUNNER=nav_indoor3_3.py
  SURVEY_LAUNCH=indoor3_2_camera.launch
fi
if [[ "$SURVEY_VARIANT" == indoor3_2 ]]; then SURVEY_FLAGS+=(--dual-camera); fi
DEFAULT_MAP="$HOME/test_data/mapping/20261114_101654_indoor_fly_raw_000001"
DEFAULT_RELOC_CONFIG="$INDOOR_SCRIPT_DIR/../src/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$INDOOR_SCRIPT_DIR/../src/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_NAV_CONFIG="$INDOOR_SCRIPT_DIR/nav_test/indoor_fly_nav.yaml"
DEFAULT_EV_MODE=odom
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3

test_desc() {
  if [[ "$SURVEY_VARIANT" == indoor3_3 ]]; then
    echo "Tag 1 前方 0.8 m 双相机扫码"
  else
    echo "货架四点拍照识别 AprilTag、写入 CSV、右下悬停"
  fi
}
test_arg() { return 1; }
test_preflight() {
  [ "$EV_MODE" = odom ] && [ "$NAV_CONFIG" -ef "$DEFAULT_NAV_CONFIG" ] && [ "$MAP_FRAME" -ef "$DEFAULT_MAP_FRAME" ] ||
    { echo "$TEST_NAME 需要飞扫图的导航配置、地图标定和 odom 模式" >&2; return 1; }
  python3 -B "$INDOOR_SCRIPT_DIR/nav_test/nav_indoor3.py" --verify-map "$MAP"
}

test_after_fusion() {
  local topics
  topics="$(timeout 5 rostopic list)" || return 1
  if [[ "$SURVEY_VARIANT" != indoor3_1 ]] && grep -Eq '^(/usb_cam/(image_raw|camera_info)|/ov9281_apriltag_vision/survey)$' <<<"$topics"; then
    err "OV9281 相机或检测器已被占用"
    return 1
  fi
  if grep -Eq '^(/ov13855(_full)?/image_raw|/ov13855_apriltag_vision/(detections|survey)|/front_marker/pose|/barcode_reader_camera/result|/indoor4/target_tag_pose)$' <<<"$topics"; then
    err "OV13855 或标签视觉链路已有节点，请在地面结束原相机任务后再启动 $TEST_NAME"
    return 1
  fi
  PF_SURVEY_CAMERA="$RUN_DIR/${SURVEY_VARIANT}_camera.pid"
  start_child "$PF_SURVEY_CAMERA" "$OUT/camera.log" roslaunch "$NT_DIR/$SURVEY_LAUNCH" "${CAMERA_ARGS[@]}"
  python3 -B "$NT_DIR/$SURVEY_RUNNER" "${SURVEY_FLAGS[@]}" --probe-camera --out "$OUT" 2>&1 | tee -a "$LOG"
}

test_cleanup() {
  [ -z "${PF_SURVEY_CAMERA:-}" ] || stop_child "$PF_SURVEY_CAMERA" "$TEST_NAME 相机与 AprilTag 检测" INT 10
}

run_test() {
  nt_check_chain || return 1
  local flags=()
  if [ "$PREVIEW" -eq 1 ]; then
    local answer
    read -r -p "回车开始单圈预览，q 退出：" answer </dev/tty || return
    [ -z "$answer" ] || return
    flags+=(--preview)
  fi
  if [[ "$SURVEY_VARIANT" == indoor3_3 ]]; then
    log "$TEST_NAME：人工起飞悬停，等设定值预发送后切 OFFBOARD，原地锁定 Tag 1"
  else
    log "$TEST_NAME：人工起飞悬停，等设定值预发送后切 OFFBOARD，从当前位置规划到 1 号点"
  fi
  python3 -B "$NT_DIR/$SURVEY_RUNNER" "${SURVEY_FLAGS[@]}" --out "$OUT" "${flags[@]}" > >(trap '' INT; exec tee -a "$LOG") 2>&1
}

source "$INDOOR_SCRIPT_DIR/nav_test/common.sh"
