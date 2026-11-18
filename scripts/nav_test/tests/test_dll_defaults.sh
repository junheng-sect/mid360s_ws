#!/usr/bin/env bash
# Exercise the public policy option without a ROS master or a running task.
set -euo pipefail
scripts=$(cd "$(dirname "$0")/../.." && pwd)
bash "$scripts/nav_test_indoor3_1.sh" --show-dll-config | grep -F 'DLL 旁路：开启'
bash "$scripts/nav_test_indoor3_1.sh" --show-dll-config --no-dll | grep -F 'DLL 旁路：关闭'
bash "$scripts/nav_test_indoor3_1.sh" --show-dll-config --record-raw-lidar | grep -F '原始雷达：开启'
source "$scripts/nav_test/common.sh"
for task in test{1..11} test_indoor{1..4}; do
  if nt_default_dll "$task"; then echo "unexpected default: $task" >&2; exit 1; fi
done
nt_default_dll test13
nt_default_dll test_indoor5
