# shellcheck shell=bash
# source 本文件：常驻 roscore / MAVROS（systemd 服务，见同目录 install.sh）的名字、MAVROS 本地口和检查函数，
# fusion.sh / start_handheld_test.sh 共用。这两个服务开机就起，入口脚本只用不起不停。
RESIDENT_ROSCORE_UNIT="mid360s-roscore.service"
RESIDENT_MAVROS_UNIT="mid360s-mavros.service"
# MAVROS 的本地口（mavros.launch 的 gcs_url，只绑 127.0.0.1，quiet mode）：pymavlink 用 udpout 连它
FC_LOCAL_URL="udpout:127.0.0.1:14555"

# resident_proc PID：该进程属于常驻服务。按 cgroup 认：roslaunch 用 setsid 起的节点也留在服务的 cgroup 里
resident_proc() { grep -qE '/mid360s-(roscore|mavros)\.service$' "/proc/$1/cgroup" 2>/dev/null; }

# mavros_pid：常驻服务里 mavros_node 的 PID；不在运行时什么都不输出、返回 1
mavros_pid() {
  local _p
  for _p in $(pgrep -x mavros_node 2>/dev/null); do
    resident_proc "$_p" && { printf '%s\n' "$_p"; return 0; }
  done
  return 1
}

unit_active() { systemctl is-active --quiet "$1"; }
