#!/usr/bin/env bash
# ==============================================================================
# 室内 test3_3：切入 OFFBOARD 后由 OV13855 锁定 Tag 1，前往 OV13855 光心距 Tag 中心 0.80 m 的扫码点，地图航向 -89.5°，稳定到位后最多读 2 s，首次读出 BOX-01 即结束本轮。
# 板端运行 ~/mid360s_ws/scripts/nav_test_indoor3_3.sh，人工定点起飞、切 OFFBOARD，完成后悬停，切出再切入可重新测试。
# 默认随通道 6 录轻量 DLL 旁路对照。需要原始雷达加 --record-raw-lidar，单 FAST-LIO2 加 --no-dll。
# 手持预览加 --preview。每轮结果写入 round_NNN/round.json 和 rounds.csv。
# ==============================================================================
TEST_NAME=test_indoor3_3
INDOOR_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$INDOOR_SCRIPT_DIR/nav_test/indoor3_survey.sh"
nav_test_main "$@"
