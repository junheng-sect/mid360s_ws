#!/usr/bin/env bash
set -e
root=/home/orangepi/mid360s_ws/scripts
source /opt/ros/noetic/setup.bash
source "$root/ros_net.sh"
export PYTHONPATH="/home/orangepi/.local/share/mid360s-ulog-export/vendor:$root:${PYTHONPATH:-}"
exec /usr/bin/python3 -B -m flight_archive.exporter
