#!/usr/bin/env bash
# 本机 Docker RViz 入口；实现和配置均随本仓库跟踪，不依赖旧的外部目录。
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$WS_DIR/docker/ros-noetic-rviz/scripts/run_rviz.sh" "$@"
