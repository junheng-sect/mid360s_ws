#!/usr/bin/env bash
# 常驻 MAVROS：systemd mid360s-mavros.service 的 ExecStart（安装见同目录 install.sh）。
# 走 scripts/launch/mavros.launch 的默认参数：飞控 /dev/ttyS3:921600，另开只绑 127.0.0.1 的本地口
# udp 14555（gcs_quiet_mode，fc_set_time.py 校时走它）。mavros 节点是 required（respawn_mavros 默认 false），
# 节点一退 roslaunch 跟着退，由 systemd 重启整个服务。
# --wait：master 没起来就等，不让 roslaunch 自己另起一个 master（常驻 roscore 会因端口被占起不来）。
# 开机时和 roscore 同一刻起，ros0 地址可能还没建立：跟 run_roscore.sh 一样先等它，最多 60 s。
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for _ in $(seq 60); do
  source "$DIR/../ros_net.sh" 2>/dev/null && break
  sleep 1
done
[ "${ROS_IP:-}" = "10.250.32.1" ] || { echo "板内固定地址 10.250.32.1 60 s 内没建立（nmcli con up ros-core）" >&2; exit 1; }
source /opt/ros/noetic/setup.bash
exec roslaunch --wait "$DIR/../launch/mavros.launch"
