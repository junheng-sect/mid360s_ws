#!/usr/bin/env bash
# ==============================================================================
# 室内 test3：货架左下 → 左上 → 右上 → 右下，每点停 3 s，自动保存 OV13855 全分辨率照片。
# 人工定点起飞，切 OFFBOARD 后延时 3 s，只运行一圈。拍完右下悬停，飞手接管降落上锁。
# 从切入 OFFBOARD 后的当前位置规划到左下点，稳定对准货架后拍照。
# --preview 不向飞控发设定值，需手持或模拟到点。
# ==============================================================================
TEST_NAME=test_indoor3
INDOOR_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_MAP="$HOME/test_data/mapping/20261114_101654_indoor_fly_raw_000001"
DEFAULT_RELOC_CONFIG="$INDOOR_SCRIPT_DIR/../src/drone_relocalization/config/indoor.yaml"
DEFAULT_MAP_FRAME="$(dirname "$INDOOR_SCRIPT_DIR")/src/car_bridge/config/map_frame_indoor_fly.yaml"
DEFAULT_NAV_CONFIG="$INDOOR_SCRIPT_DIR/nav_test/indoor_fly_nav.yaml"
DEFAULT_EV_MODE=odom
DEFAULT_PREVIEW_RADIUS=0.4
DEFAULT_PREVIEW_HALF_HEIGHT=0.3

test_desc() { echo "货架四点全分辨率拍照，每点停 3 s，拍完右下悬停"; }
test_arg() { return 1; }
test_preflight() {
  [ "$EV_MODE" = odom ] && [ "$NAV_CONFIG" = "$DEFAULT_NAV_CONFIG" ] && [ "$MAP_FRAME" = "$DEFAULT_MAP_FRAME" ] ||
    { echo "室内 test3 需要飞扫图的导航配置、地图标定和 odom 模式" >&2; return 1; }
  python3 -B "$INDOOR_SCRIPT_DIR/nav_test/nav_indoor3.py" --verify-map "$MAP"
}

# fusion.sh 的启动检查要求此时还没有本任务的相机 roslaunch。
test_after_fusion() {
  PF_INDOOR3_CAMERA="$RUN_DIR/indoor3_camera.pid"
  local publishers
  publishers="$(timeout 5 rostopic list)" || return 1
  if grep -qx '/ov13855_full/image_raw' <<<"$publishers" ||
     timeout 5 rosnode list | grep -qx '/indoor3_full_cam'; then
    err "全分辨率相机已有节点，请在地面结束原相机任务后再启动 test3"
    return 1
  fi
  start_child "$PF_INDOOR3_CAMERA" "$OUT/camera.log" roslaunch "$NT_DIR/indoor3_camera.launch"
  python3 -B "$NT_DIR/nav_indoor3.py" --probe-camera --out "$OUT" 2>&1 | tee -a "$LOG"
}

test_cleanup() {
  [ -z "${PF_INDOOR3_CAMERA:-}" ] || stop_child "$PF_INDOOR3_CAMERA" "test3 相机" INT 10
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
  log "test3：人工起飞悬停，等设定值预发送后切 OFFBOARD，从当前位置规划到 1 号点"
  python3 -B "$NT_DIR/nav_indoor3.py" --out "$OUT" "${flags[@]}" > >(trap '' INT; exec tee -a "$LOG") 2>&1
}

source "$INDOOR_SCRIPT_DIR/nav_test/common.sh"
nav_test_main "$@"
