#!/usr/bin/env bash
# 本机起无人机状态桥（local_view/drone_status.py）：在 docker 里只读订阅板端常驻 MAVROS 和 fusion 的几个低频话题，
# 汇总成 JSON 给网页（tools/car_web 的无人机面板）取：http://<本机 IP>:8091/status。板端不加任何进程。
#
#   ./scripts/run_drone_status.sh            # 后台起（容器 drone_status，已有同名容器先替换）
#   ./scripts/run_drone_status.sh --fg       # 前台起，看日志用
#   ./scripts/run_drone_status.sh --stop     # 停掉
#   ./scripts/run_drone_status.sh --port 8091 --ros-master URI --ros-ip IP
#
# 不加 --restart：本机 ROS_IP 在起容器时定下，换网后要重跑本脚本。板端重启（master 换了）不用管，节点自己会重来。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

name="drone_status"
port=8091
ros_master_uri="${RVIZ_ROS_MASTER:-}"
ros_ip="${RVIZ_ROS_IP:-}"
foreground=0

while [ $# -gt 0 ]; do
  case "$1" in
    --fg) foreground=1; shift ;;
    --stop) docker rm -f "$name" >/dev/null 2>&1 && log "已停掉 $name" || log "$name 本来就没在跑"; exit 0 ;;
    --port) port="$2"; shift 2 ;;
    --ros-master) ros_master_uri="$2"; shift 2 ;;
    --ros-ip) ros_ip="$2"; shift 2 ;;
    -h|--help) sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) die "未知参数：$1（-h 看用法）" ;;
  esac
done

check_docker
check_image
[ -n "$ros_master_uri" ] || ros_master_uri="$(resolve_ros_master)"
master_host="$(printf '%s' "$ros_master_uri" | sed -E 's#^[a-z]+://([^:/]+).*#\1#')"
if [ "$master_host" = "$BOARD_CORE_IP" ]; then
  ensure_core_route "$DEFAULT_BOARD_IP" \
    || die "到板内地址 $BOARD_CORE_IP 的路由没配上（板端 Wi-Fi 地址：${DEFAULT_BOARD_IP:-没找到}）：确认本机和板端连在同一个 Wi-Fi。"
fi
if [ -z "$ros_ip" ]; then
  ros_ip="$(detect_ros_ip "$DEFAULT_BOARD_IP")" || die "无法确定到板端的本机源地址：确认本机和板端连在同一个 Wi-Fi。"
fi
# 板端没开也照样起：页面会显示“连不上板端”，板端起来后自动连上
if pid="$(check_ros_master "$ros_master_uri" 2>/dev/null)"; then
  log "板端 master 可达（rosmaster pid=$pid）"
else
  warn "板端 master 现在连不上（$ros_master_uri），状态桥会一直等它"
fi
log "ROS_MASTER_URI=$ros_master_uri  ROS_IP=$ros_ip  端口 $port"

docker rm -f "$name" >/dev/null 2>&1 || true
mkdir -p "$CONFIG_DIR/home/ros"
docker_args=(
  run --rm --init
  --name "$name"
  --network host
  --user "$(id -u):$(id -g)"
  -e ROS_MASTER_URI="$ros_master_uri"
  -e ROS_IP="$ros_ip"
  -e PYTHONUNBUFFERED=1
  -v "$CONFIG_DIR:/opt/rviz-ws:ro"
  -v "$CONFIG_DIR/home/ros:/home/ros"
  -w /home/ros
)
cmd=(python3 /opt/rviz-ws/local_view/scripts/drone_status.py --port "$port")
if [ "$foreground" -eq 1 ]; then
  exec docker "${docker_args[@]}" "$IMAGE_TAG" "${cmd[@]}"
fi
docker "${docker_args[@]}" -d "$IMAGE_TAG" "${cmd[@]}" >/dev/null
log "已后台起 $name：curl http://127.0.0.1:$port/status；日志 docker logs -f $name；停 $0 --stop"
