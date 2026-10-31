#!/usr/bin/env bash
# 本机起无人机状态桥（local_view/drone_status.py）：在 docker 里只读订阅板端常驻 MAVROS 和 fusion 的几个低频话题，
# 汇总成 JSON 给网页（tools/car_web 的无人机面板）取：http://<本机 IP>:8091/status。板端不加任何进程。
#
#   ./scripts/run_drone_status.sh --install    # 装成本机用户服务 drone-status：开机自启、退出 10 s 后重起（平时用这个）
#   ./scripts/run_drone_status.sh --uninstall  # 卸掉服务
#   ./scripts/run_drone_status.sh              # 不装服务时：后台起（容器 drone_status，已有同名容器先替换）
#   ./scripts/run_drone_status.sh --fg         # 前台起，看日志用（服务也是这样跑的）
#   ./scripts/run_drone_status.sh --stop       # 停掉
#   ./scripts/run_drone_status.sh --port 8091 --ros-master URI --ros-ip IP
#
# 本机 ROS_IP 在起容器时按到板端的路由定下。自动识别时，状态桥发现本机到板端的源地址变了（换网）会自己退出，
# 服务重起时重选；手动起的要重跑本脚本。板端重启（master 换了）不用管，节点自己会重来。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

name="drone_status"
unit="drone-status.service"
unit_file="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/$unit"
port=8091
ros_master_uri="${RVIZ_ROS_MASTER:-}"
ros_ip="${RVIZ_ROS_IP:-}"
action=background

while [ $# -gt 0 ]; do
  case "$1" in
    --fg) action=foreground; shift ;;
    --stop) action=stop; shift ;;
    --install) action=install; shift ;;
    --uninstall) action=uninstall; shift ;;
    --port) port="$2"; shift 2 ;;
    --ros-master) ros_master_uri="$2"; shift 2 ;;
    --ros-ip) ros_ip="$2"; shift 2 ;;
    -h|--help) sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) die "未知参数：$1（-h 看用法）" ;;
  esac
done

# 服务跑的是 --fg：每次起都重新选 ROS_IP、补路由；本机和板端不在一个网时起不来，每 10 s 重试。
# 服务没有终端输 sudo 密码，要 sudo 补路由的网（手机热点等）得先手动跑一次本脚本或 run_rviz.sh。
install_unit() {
  local docker_bin
  docker_bin="$(command -v docker)" || die "未找到 docker 命令。"
  mkdir -p "$(dirname "$unit_file")"
  cat >"$unit_file" <<EOF
[Unit]
Description=无人机状态桥（mid360s_ws docker/ros-noetic-rviz，:$port/status，给 car_web 无人机面板）

[Service]
Environment=DRONE_STATUS_SERVICE=1
ExecStart="$SCRIPT_DIR/run_drone_status.sh" --fg --port $port
ExecStopPost=-$docker_bin rm -f $name
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
EOF
  docker rm -f "$name" >/dev/null 2>&1 || true   # 换下手动起的那个
  systemctl --user daemon-reload
  systemctl --user enable "$unit"
  systemctl --user restart "$unit"
  log "已装成用户服务 $unit（$unit_file），开机自启"
  [ "$(loginctl show-user "$(id -un)" -p Linger --value 2>/dev/null)" = yes ] \
    || warn "当前用户没开 linger：要登录桌面后服务才起；开机就起先执行 sudo loginctl enable-linger $(id -un)"
  log "看状态 systemctl --user status $unit；日志 journalctl --user -u $unit -f"
}

case "$action" in
  install) install_unit; exit 0 ;;
  uninstall)
    systemctl --user disable --now "$unit" 2>/dev/null || true
    rm -f "$unit_file"
    systemctl --user daemon-reload
    log "已卸掉用户服务 $unit"
    exit 0 ;;
esac
# 服务在跑时手动起停会和它抢同名容器（服务 10 s 后又把手动起的替换掉），改用 systemctl --user。
# 服务自己由单元里的 DRONE_STATUS_SERVICE=1 认出来（INVOCATION_ID 不行：桌面里开的终端也继承了它）
if [ "${DRONE_STATUS_SERVICE:-}" != 1 ] && systemctl --user is-active -q "$unit" 2>/dev/null; then
  die "状态桥已由用户服务 $unit 管着：重启 systemctl --user restart $unit，停 systemctl --user stop $unit，日志 journalctl --user -u $unit -f"
fi
if [ "$action" = stop ]; then
  docker rm -f "$name" >/dev/null 2>&1 && log "已停掉 $name" || log "$name 本来就没在跑"
  exit 0
fi

check_docker
check_image
[ -n "$ros_master_uri" ] || ros_master_uri="$(resolve_ros_master)"
master_host="$(printf '%s' "$ros_master_uri" | sed -E 's#^[a-z]+://([^:/]+).*#\1#')"
if [ "$master_host" = "$BOARD_CORE_IP" ]; then
  ensure_core_route "$DEFAULT_BOARD_IP" \
    || die "到板内地址 $BOARD_CORE_IP 的路由没配上（板端 Wi-Fi 地址：${DEFAULT_BOARD_IP:-没找到}）：确认本机和板端连在同一个 Wi-Fi。"
fi
auto_ip=0
if [ -z "$ros_ip" ]; then
  ros_ip="$(detect_ros_ip "$DEFAULT_BOARD_IP")" || die "无法确定到板端的本机源地址：确认本机和板端连在同一个 Wi-Fi。"
  auto_ip=1
fi
# 板端没开也照样起：页面会显示“连不上板端”，板端起来后自动连上
if pid="$(check_ros_master "$ros_master_uri" 2>/dev/null)"; then
  log "板端 master 可达（rosmaster pid=$pid）"
else
  warn "板端 master 现在连不上（$ros_master_uri），状态桥会一直等它"
fi
log "ROS_MASTER_URI=$ros_master_uri  ROS_IP=$ros_ip$([ "$auto_ip" -eq 1 ] && printf '（自动识别）')  端口 $port"

docker rm -f "$name" >/dev/null 2>&1 || true
mkdir -p "$CONFIG_DIR/home/ros"
docker_args=(
  run --rm --init
  --name "$name"
  --network host
  --user "$(id -u):$(id -g)"
  -e ROS_MASTER_URI="$ros_master_uri"
  -e ROS_IP="$ros_ip"
  -e DRONE_STATUS_AUTO_IP="$auto_ip"
  -e PYTHONUNBUFFERED=1
  -v "$CONFIG_DIR:/opt/rviz-ws:ro"
  -v "$CONFIG_DIR/home/ros:/home/ros"
  -w /home/ros
)
cmd=(python3 /opt/rviz-ws/local_view/scripts/drone_status.py --port "$port")
if [ "$action" = foreground ]; then
  exec docker "${docker_args[@]}" "$IMAGE_TAG" "${cmd[@]}"
fi
docker "${docker_args[@]}" -d "$IMAGE_TAG" "${cmd[@]}" >/dev/null
log "已后台起 $name：curl http://127.0.0.1:$port/status；日志 docker logs -f $name；停 $0 --stop；开机自启用 $0 --install"
