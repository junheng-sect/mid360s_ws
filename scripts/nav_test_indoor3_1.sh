#!/usr/bin/env bash
# ==============================================================================
# 室内 test3_1：货架左下 → 左上 → 右上 → 右下，每点稳住 1 秒后观察 AprilTag 2 秒并拍照。
# 板端运行 ~/mid360s_ws/scripts/nav_test_indoor3_1.sh，人工定点起飞、切 OFFBOARD，右下由飞手接管。
# 手持预览加 --preview。每点每个 ID 保留一行，见任务目录 apriltag_observations.csv。
# ==============================================================================
TEST_NAME=test_indoor3_1
INDOOR_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_MAP="$HOME/test_data/mapping/20261114_101654_indoor_fly_raw_000001"
DEFAULT_RELOC_CONFIG="$INDOOR_SCRIPT_DIR/../src/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$INDOOR_SCRIPT_DIR/../src/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_NAV_CONFIG="$INDOOR_SCRIPT_DIR/nav_test/indoor_fly_nav.yaml"
DEFAULT_EV_MODE=odom
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3

test_desc() { echo "货架四点拍照识别 AprilTag、写入 CSV、右下悬停"; }
test_arg() { return 1; }
test_preflight() {
  [ "$EV_MODE" = odom ] && [ "$NAV_CONFIG" = "$DEFAULT_NAV_CONFIG" ] && [ "$MAP_FRAME" = "$DEFAULT_MAP_FRAME" ] ||
    { echo "室内 test3_1 需要飞扫图的导航配置、地图标定和 odom 模式" >&2; return 1; }
  python3 -B "$INDOOR_SCRIPT_DIR/nav_test/nav_indoor3.py" --verify-map "$MAP"
}

test_after_fusion() {
  local topics
  topics="$(timeout 5 rostopic list)" || return 1
  if grep -Eq '^(/ov13855(_full)?/image_raw|/ov13855_apriltag_vision/(detections|survey)|/front_marker/pose|/barcode_reader_camera/result|/indoor4/target_tag_pose)$' <<<"$topics"; then
    err "OV13855 或标签视觉链路已有节点，请在地面结束原相机任务后再启动 test3_1"
    return 1
  fi
  PF_INDOOR3_1_CAMERA="$RUN_DIR/indoor3_1_camera.pid"
  start_child "$PF_INDOOR3_1_CAMERA" "$OUT/camera.log" roslaunch "$NT_DIR/indoor3_1_camera.launch"
  python3 -B "$NT_DIR/nav_indoor3_1.py" --probe-camera --out "$OUT" 2>&1 | tee -a "$LOG"
}

test_cleanup() {
  [ -z "${PF_INDOOR3_1_CAMERA:-}" ] || stop_child "$PF_INDOOR3_1_CAMERA" "test3_1 相机与 AprilTag 检测" INT 10
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
  log "test3_1：人工起飞悬停，等设定值预发送后切 OFFBOARD，从当前位置规划到 1 号点"
  python3 -B "$NT_DIR/nav_indoor3_1.py" --out "$OUT" "${flags[@]}" > >(trap '' INT; exec tee -a "$LOG") 2>&1
}

source "$INDOOR_SCRIPT_DIR/nav_test/common.sh"
nav_test_main "$@"
