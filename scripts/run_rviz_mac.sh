#!/usr/bin/env bash
# Mac-only browser RViz entry. The original run_rviz.sh stays for Ubuntu.
set -euo pipefail
WS_DIR="$(python3 -c 'import os,sys; print(os.path.dirname(os.path.dirname(os.path.realpath(sys.argv[1]))))' "${BASH_SOURCE[0]}")"
exec "$WS_DIR/docker/ros-noetic-rviz/scripts/run_rviz_mac.sh" "$@"
