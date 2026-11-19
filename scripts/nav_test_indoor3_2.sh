#!/usr/bin/env bash
# ==============================================================================
# 室内 test3_2：货架左下 → 左上 → 右上 → 右下，每点稳住 1 秒后用 OV13855 和 OV9281 同时观察 AprilTag 2 秒，每台各拍一张。
# 板端运行 ~/mid360s_ws/scripts/nav_test_indoor3_2.sh，人工定点起飞、切 OFFBOARD，右下由飞手接管。
# 默认随通道 6 录轻量 DLL 旁路对照。需要原始雷达加 --record-raw-lidar，单 FAST-LIO2 加 --no-dll。
# 手持预览加 --preview。每台相机、每点、每个 ID 保留一行，见任务目录 apriltag_observations.csv。
# ==============================================================================
TEST_NAME=test_indoor3_2
INDOOR_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$INDOOR_SCRIPT_DIR/nav_test/indoor3_survey.sh"
nav_test_main "$@"
