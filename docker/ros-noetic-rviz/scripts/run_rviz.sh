#!/usr/bin/env bash
# 本机一条命令启动 RViz：检查环境 -> 推导 ROS 网络参数 -> 以宿主机用户身份跑容器。
#
#   ./scripts/run_rviz.sh                     # 打开 RViz（drone_odom 重力对齐视图），订阅 /cloud_registered
#   ./scripts/run_rviz.sh --shell             # 进入同一环境的交互式 shell
#   ./scripts/run_rviz.sh --cmd 'rostopic hz /cloud_registered'
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

usage() {
  cat <<'EOF'
用法: run_rviz.sh [选项]

  --config PATH       容器内 RViz 配置路径（默认本机 home/ros/.rviz/fastlio_mid360s.rviz）
  --reloc             打开重定位对照视图（个人副本 home/ros/.rviz/drone_reloc.rviz，首次从
                      rviz/drone_reloc.rviz 复制）：Fixed Frame map，先验地图、实时点云、map/车/无人机三组
                      坐标轴和二维车图；本机同时起显示辅助节点 local_view/reloc_display（板端不跑显示节点）
  --ros-master URI    ROS Master 地址（默认 http://192.168.31.32:11311）
  --ros-ip IP         覆盖自动识别的本机 ROS IP
  --name NAME         容器名（默认 rviz_mid360s，同名容器会自动清理）
  --shell             启动 bash 而不是 RViz
  --cmd 'COMMAND'     在容器内执行命令（可直接用 rostopic/rosrun 等）
  --no-gpu            不传 --gpus all（纯话题检查时可省去 GPU 依赖）
  --no-master-check   跳过 ROS Master 连通性检查
  --detach            后台运行（容器名固定，便于 docker logs / docker stop）
  --no-maximize       兼容旧参数；现在默认不自动最大化，仍检查左侧面板
  --maximize-delay N  已弃用，接受该参数但不再改变启动行为
  --dry-run           只打印将要执行的 docker 命令
  -h, --help          显示本帮助

默认模式用 roslaunch 打开 RViz（Fixed Frame = drone_odom，drone_odom -> camera_init 静态 TF 由板端
fastlio_gravity_bridge 节点发布；板端没跑时 RViz 会提示该 frame 不存在）。
停止后台容器：docker stop <容器名>（默认 rviz_mid360s）
左侧面板出现黑块/错位时（仓库根目录）：./docker/ros-noetic-rviz/scripts/repair_layout.sh
EOF
}

# 默认打开被 Git 忽略的个人副本，保存布局不会改动公共模板；--reloc 换成重定位对照模板。
rviz_template="fastlio_mid360s.rviz"
reloc=0
rviz_config=""
ros_master_uri="${RVIZ_ROS_MASTER:-}"
ros_ip_override="${RVIZ_ROS_IP:-}"
run_mode="rviz"
run_cmd=""
use_gpu=1
master_check=1
dry_run=0
detach=0
panel_check_delay=6

while [ $# -gt 0 ]; do
  case "$1" in
    --config) rviz_config="$2"; shift 2 ;;
    --reloc) rviz_template="drone_reloc.rviz"; reloc=1; shift ;;
    --ros-master) ros_master_uri="$2"; shift 2 ;;
    --ros-ip) ros_ip_override="$2"; shift 2 ;;
    --name) CONTAINER_NAME="$2"; shift 2 ;;
    --shell) run_mode="shell"; shift ;;
    --cmd) run_mode="cmd"; run_cmd="$2"; shift 2 ;;
    --no-gpu) use_gpu=0; shift ;;
    --no-master-check) master_check=0; shift ;;
    --detach) detach=1; shift ;;
    --no-maximize) shift ;;
    --maximize-delay)
      [ "$#" -ge 2 ] || die "--maximize-delay 缺少参数"
      [[ "$2" =~ ^[0-9]+$ ]] || die "--maximize-delay 需要非负整数"
      warn "--maximize-delay 已弃用：不再自动最大化，此参数将被忽略"
      shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "未知参数：$1" ;;
  esac
done

personal_config="$CONFIG_DIR/home/ros/.rviz/$rviz_template"
personal_config_in_container="/home/ros/.rviz/$rviz_template"
# 优先级：--config > --reloc > local.env 的 RVIZ_CONFIG > 默认个人副本。
if [ -z "$rviz_config" ]; then
  if [ "$reloc" -eq 1 ]; then rviz_config="$personal_config_in_container"
  else rviz_config="${RVIZ_CONFIG:-$personal_config_in_container}"; fi
fi

[ -n "$ros_master_uri" ] || ros_master_uri="$(resolve_ros_master)"

log "1/5 检查 Docker"
check_docker
check_image
if [ "$use_gpu" -eq 1 ]; then check_nvidia_runtime; fi
if [ "$reloc" -eq 1 ]; then
  # 重定位视图换成带本机显示辅助节点的 launch。
  LAUNCH_FILE_IN_CONTAINER="/opt/rviz-ws/launch/display_reloc.launch"
  LAUNCH_FILE_ON_HOST="$CONFIG_DIR/launch/display_reloc.launch"
fi
if [ "$run_mode" = rviz ]; then check_launch_file; fi

log "2/5 检查 GPU"
if [ "$use_gpu" -eq 1 ]; then
  check_gpu_host >&2
else
  warn "已按 --no-gpu 跳过 GPU 检查"
fi

log "3/5 检查图形显示"
check_display_env
xauth_file="$CONFIG_DIR/runtime/.docker.xauth"
have_xauth=0
if make_xauth_file; then have_xauth=1; fi
ensure_xhost_grant

log "4/5 确定 ROS 网络参数"
if [ -n "$ros_ip_override" ]; then
  ros_ip="$ros_ip_override"
else
  board_ip="$(printf '%s' "$ros_master_uri" | sed -E 's#^[a-z]+://([^:/]+).*#\1#')"
  case "$board_ip" in
    ''|*[!0-9.]*) board_ip="$DEFAULT_BOARD_IP" ;;
  esac
  ros_ip="$(detect_ros_ip "$board_ip")" \
    || die "无法确定到 $board_ip 的本机源地址：确认 Wi-Fi 已连接且在 192.168.31.0/24 网段。"
fi
log "ROS_MASTER_URI=$ros_master_uri  ROS_IP=$ros_ip"
if [ "$master_check" -eq 1 ]; then
  if pid="$(check_ros_master "$ros_master_uri")"; then
    log "ROS Master 可达（rosmaster pid=$pid）"
  else
    die "ROS Master 不可达：$ros_master_uri。请先在板端启动 roscore 或 roslaunch（板端命令见 README.md）。"
  fi
else
  warn "已按 --no-master-check 跳过 ROS Master 检查"
fi

log "5/5 启动容器 $CONTAINER_NAME"

docker_args=(
  run
  --name "$CONTAINER_NAME"
  --network host
  # Qt5/XCB 与宿主机 X11 共享 IPC，避免停靠面板滚动残影（rviz#1628/#1842）。
  --ipc=host
  --hostname "$(hostname)"
  --user "$(id -u):$(id -g)"
  -e DISPLAY="$DISPLAY"
  -e QT_X11_NO_MITSHM=1
  # 关闭 RViz 启动时的 “ROS 1 End-of-Life” 提示框，避免遮挡视图。
  -e DISABLE_ROS1_EOL_WARNINGS=1
  -e ROS_MASTER_URI="$ros_master_uri"
  -e ROS_IP="$ros_ip"
  # 不设置 ROS_HOSTNAME：留空字符串会让 rospy 报 "invalid ROS_HOSTNAME"，
  # 由 ROS_IP 决定本机节点地址即可。
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "$CONFIG_DIR:/opt/rviz-ws:ro"
  -v "$CONFIG_DIR/home/ros:/home/ros"
  -v "$CONFIG_DIR/logs:/logs"
  -w /home/ros
)
if [ "$detach" -eq 1 ]; then
  # 后台运行：保留容器（不加 --rm），出错后可用 docker logs 查看原因。
  docker_args+=(-d)
elif [ -t 0 ] && [ -t 1 ]; then
  docker_args+=(--rm -it)
else
  docker_args+=(--rm)
fi
if [ "$use_gpu" -eq 1 ]; then
  docker_args+=(
    --gpus all
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display
  )
fi
if [ "$have_xauth" -eq 1 ]; then
  docker_args+=(-e XAUTHORITY=/tmp/.docker.xauth -v "$xauth_file:/tmp/.docker.xauth:ro")
fi

case "$run_mode" in
  # 显示模式：roslaunch 打开 RViz（required="true"，关窗/Ctrl+C/docker stop 时退出）。
  # drone_odom -> camera_init 静态 TF 由板端 fastlio_gravity_bridge 发布，本机不发布。
  # --config 只作为 config 参数透传，不改用户给的配置。
  rviz)
    if [ "$reloc" -eq 1 ]; then
      # local_view 不编译：入口脚本 source 过 ROS 之后再把它加进 ROS_PACKAGE_PATH，roslaunch 才找得到 reloc_display.py。
      set -- bash -c 'export ROS_PACKAGE_PATH=/opt/rviz-ws/local_view:$ROS_PACKAGE_PATH; exec roslaunch "$0" "config:=$1"' \
        "$LAUNCH_FILE_IN_CONTAINER" "$rviz_config"
    else
      set -- roslaunch "$LAUNCH_FILE_IN_CONTAINER" "config:=$rviz_config"
    fi ;;
  shell) set -- bash ;;
  cmd)   set -- bash -lc "$run_cmd" ;;
esac

# 记录启动前已有的 RViz 窗口，稍后据此识别本次新开的窗口，避免误操作用户之前那扇窗。
# 只检查本次新窗口的左侧面板，不改变主窗口尺寸或最大化状态。
rviz_windows_before=""
have_window_tools=1
if [ "$run_mode" = rviz ]; then
  for tool in xdotool xwininfo xprop python3; do
    command -v "$tool" >/dev/null 2>&1 || have_window_tools=0
  done
  if [ "$have_window_tools" -eq 1 ]; then
    rviz_windows_before="$(list_rviz_windows | tr '\n' ' ' || true)"
  else
    warn "缺少 xdotool/xwininfo/xprop/python3，跳过面板宽度检查"
    warn "（sudo apt-get install -y xdotool x11-utils）"
  fi
fi

if [ "$dry_run" -eq 1 ]; then
  # 注意：--dry-run 必须放在任何有副作用的操作之前（例如移除同名容器），只打印计划。
  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    printf '# 已存在同名容器，实际执行时会先：docker rm -f %s\n' "$CONTAINER_NAME"
  fi
  printf 'docker'
  printf ' %q' "${docker_args[@]}" "$@"
  printf '\n'
  exit 0
fi

# 首次克隆时由当前用户创建可写目录，避免 Docker 自动建立 root 所有的挂载目录。
mkdir -p "$CONFIG_DIR/home/ros" "$CONFIG_DIR/logs"
if [ "$run_mode" = rviz ] && [ "$rviz_config" = "$personal_config_in_container" ]; then
  if [ ! -e "$personal_config" ]; then
    mkdir -p "$(dirname "$personal_config")"
    # 不覆盖另一个同时启动的入口已创建的个人配置。
    cp -n "$CONFIG_DIR/rviz/$rviz_template" "$personal_config"
    log "个人 RViz 配置：$personal_config（首次从公共模板初始化）"
  fi
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  warn "已存在同名容器，先移除：$CONTAINER_NAME"
  docker rm -f "$CONTAINER_NAME" >/dev/null
fi

if [ "$run_mode" = rviz ] && [ "$have_window_tools" -eq 1 ]; then
  # 后台等待本次新窗口布局稳定，仅在左侧面板超预算时收窄分隔条。
  # 实现与离线回放（replay_demo.sh）共用 common.sh 里的 spawn_rviz_window_watcher。
  spawn_rviz_window_watcher "$rviz_windows_before" 0 "$panel_check_delay"
fi

log "执行：docker ${docker_args[*]} $*"
exec docker "${docker_args[@]}" "$IMAGE_TAG" "$@"
