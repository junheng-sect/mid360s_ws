# Common board bring-up for indoor4/5. The public entry point selects the variant.
SCAN_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCAN_VARIANT="${TEST_NAME#test_}"
SCAN_NAME="test${SCAN_VARIANT#indoor}"
DEFAULT_MAP="$HOME/test_data/mapping/20261114_101654_indoor_fly_raw_000001"
DEFAULT_RELOC_CONFIG="$SCAN_SCRIPTS/../src/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$SCAN_SCRIPTS/../src/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_NAV_CONFIG="$SCAN_SCRIPTS/nav_test/indoor_fly_nav.yaml"
DEFAULT_EV_MODE=odom
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3

test_desc() { echo "四点识别 AprilTag、逐批扫描 QR、返回右下悬停"; }
test_arg() { return 1; }
test_preflight() {
  [ "$EV_MODE" = odom ] && [[ "$NAV_CONFIG" -ef "$DEFAULT_NAV_CONFIG" && "$MAP_FRAME" -ef "$DEFAULT_MAP_FRAME" ]] ||
    { echo "室内 ${SCAN_NAME} 需要飞扫图的导航配置、地图标定和 odom 模式" >&2; return 1; }
  python3 -B "$SCAN_SCRIPTS/nav_test/nav_indoor3.py" --verify-map "$MAP"
}

test_after_fusion() {
  local topics
  topics="$(timeout 5 rostopic list)" || return 1
  if grep -Eq '^(/ov13855(_full)?/image_raw|/ov13855_apriltag_vision/(corners|detections)|/front_marker/pose|/barcode_reader_camera/result|/indoor[45]/target_tag_pose)$' <<<"$topics"; then
    err "OV13855 或读码链路已有节点，请在地面结束原相机任务后再启动 ${SCAN_NAME}"
    return 1
  fi
  PF_SCAN_CAMERA="$RUN_DIR/${SCAN_VARIANT}_camera.pid"
  start_child "$PF_SCAN_CAMERA" "$OUT/camera.log" roslaunch "$NT_DIR/${SCAN_VARIANT}_camera.launch" save_dir:="$OUT/barcode"
  python3 -B "$NT_DIR/nav_${SCAN_VARIANT}.py" --probe-camera --out "$OUT" 2>&1 | tee -a "$LOG"
}

test_cleanup() {
  [ -z "${PF_SCAN_CAMERA:-}" ] || stop_child "$PF_SCAN_CAMERA" "${SCAN_NAME} 相机和读码" INT 10
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
  log "${SCAN_NAME}：人工起飞悬停，等设定值预发送后切 OFFBOARD，从当前位置规划到 1 号点"
  python3 -B "$NT_DIR/nav_${SCAN_VARIANT}.py" --out "$OUT" "${flags[@]}" > >(trap '' INT; exec tee -a "$LOG") 2>&1
}

source "$SCAN_SCRIPTS/nav_test/common.sh"
