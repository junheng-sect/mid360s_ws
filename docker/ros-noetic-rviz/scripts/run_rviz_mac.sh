#!/usr/bin/env bash
# Mac-only browser entry point. Ubuntu retains run_rviz.sh.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common_mac.sh"
usage() {
  cat <<'EOF'
用法: run_rviz_mac.sh [选项]
  --reloc             重定位对照视图（默认 FAST-LIO 点云视图）
  --config PATH       容器内 RViz 配置路径；默认使用本机个人副本
  --ros-master URI    默认 http://10.250.32.1:11311
  --ros-ip IP         覆盖自动识别的宿主机 ROS IP
  --name NAME         默认 rviz_mid360s
  --check             检查真实话题接收与 TF（不启动桌面，不发布控制消息）
  --stop              停止本次显示容器及其辅助服务
  --web-port PORT     Mac 浏览器端口，默认 6080；VNC 使用 PORT+10000
  --no-open           启动 Mac 显示容器，不自动打开浏览器
  --shell             进入容器 bash，不启动桌面
  --cmd COMMAND       在容器内执行命令，不启动桌面
  --no-gpu            兼容参数；Mac 固定使用软件渲染
  --no-master-check   跳过 Master 检查（仍需配置正确的网络参数）
  --detach            后台运行；Mac 显示默认后台运行并打开浏览器
  --no-maximize       兼容旧参数，不改变窗口状态
  --maximize-delay N  接受旧参数，但忽略
  --dry-run           仅打印命令；不操作 Docker、路由、授权或个人文件
  -h, --help          本帮助

Mac 首次运行须安装 Docker Desktop 4.34+ 并启用 host networking。
关 RViz 窗口或 --stop 会退出显示容器；关闭浏览器标签页不停止容器。
点云历史仍保留 1000 秒；相机使用 compressed 预览。
EOF
}
rviz_template=fastlio_mid360s.rviz
reloc=0
rviz_config=''
ros_master_uri="${RVIZ_ROS_MASTER:-}"
ros_ip_override="${RVIZ_ROS_IP:-}"
run_mode=rviz
run_cmd=''
master_check=1
dry_run=0
detach=0
web_port="${RVIZ_WEB_PORT:-6080}"
open_browser=1
while [ "$#" -gt 0 ]; do
  case "$1" in
    --config) require_value "$@"; rviz_config="$2"; shift 2 ;;
    --reloc) rviz_template=drone_reloc.rviz; reloc=1; shift ;;
    --ros-master) require_value "$@"; ros_master_uri="$2"; shift 2 ;;
    --ros-ip) require_value "$@"; ros_ip_override="$2"; shift 2 ;;
    --name) require_value "$@"; CONTAINER_NAME="$2"; shift 2 ;;
    --web-port) require_value "$@"; web_port="$2"; shift 2 ;;
    --shell) run_mode=shell; shift ;;
    --cmd) require_value "$@"; run_mode=cmd; run_cmd="$2"; shift 2 ;;
    --check) run_mode=check; shift ;;
    --stop) run_mode=stop; shift ;;
    --no-gpu) shift ;;
    --no-open) open_browser=0; shift ;;
    --no-master-check) master_check=0; shift ;;
    --detach) detach=1; shift ;;
    --no-maximize) shift ;;
    --maximize-delay)
      require_value "$@"; [[ "$2" =~ ^[0-9]+$ ]] || die "--maximize-delay 需要非负整数"
      warn "--maximize-delay 已弃用，忽略"; shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数：$1（--help 看用法）" ;;
  esac
done
if [ "$run_mode" = stop ]; then stop_display; exit; fi
[ -n "$ros_master_uri" ] || ros_master_uri="$(resolve_ros_master)"
personal_config_in_container="/home/ros/.rviz/$rviz_template"
if [ -z "$rviz_config" ]; then
  if [ "$reloc" -eq 1 ]; then rviz_config="$personal_config_in_container"
  else rviz_config="${RVIZ_CONFIG:-$personal_config_in_container}"; fi
fi
personal_config=''
[ "$rviz_config" != "$personal_config_in_container" ] || personal_config="$CONFIG_DIR/home/ros/.rviz/$rviz_template"
display_template="$CONFIG_DIR/rviz/$rviz_template"
display_home="$CONFIG_DIR/home/ros"
display_logs="$CONFIG_DIR/logs"
extra_mounts=(-v "$CONFIG_DIR:/opt/rviz-ws:ro")
check_mode=fastlio
fixed_frame=drone_odom
if [ "$reloc" -eq 1 ]; then
  LAUNCH_FILE_IN_CONTAINER=/opt/rviz-ws/launch/display_reloc_mac.launch
  LAUNCH_FILE_ON_HOST="$CONFIG_DIR/launch/display_reloc_mac.launch"
  check_mode=reloc
  fixed_frame=map
fi
case "$run_mode" in
  rviz)
    check_launch_file
    display_command=(bash -c 'export ROS_PACKAGE_PATH=/opt/rviz-ws/local_view:$ROS_PACKAGE_PATH; exec roslaunch "$0" "config:=$1"'
      "$LAUNCH_FILE_IN_CONTAINER" "$rviz_config") ;;
  shell) display_command=(bash) ;;
  cmd) display_command=(bash -c "$run_cmd") ;;
  check)
    detach=0
    CONTAINER_NAME="${CONTAINER_NAME}_check_$$"
    display_command=(python3 /opt/rviz-ws/scripts/check_inputs.py --mode "$check_mode" --fixed-frame "$fixed_frame") ;;
esac
# Each Mac desktop needs its own ROS name, even across separate containers.
if [ "$run_mode" = rviz ]; then
  display_command=(bash -c 'export ROS_PACKAGE_PATH=/opt/rviz-ws/local_view:$ROS_PACKAGE_PATH; exec roslaunch "$0" "config:=$1" "rviz_name:=$2"'
    "$LAUNCH_FILE_IN_CONTAINER" "$rviz_config" "mac_rviz_${CONTAINER_NAME//[^a-zA-Z0-9_]/_}")
fi
run_display
