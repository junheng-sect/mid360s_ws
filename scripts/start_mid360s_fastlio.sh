#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# 一键前台启动 Livox MID-360S 驱动、FAST-LIO2 与重力对齐桥接
# 路径: /home/orangepi/mid360s_ws/scripts/start_mid360s_fastlio.sh
# 用法: ./start_mid360s_fastlio.sh [--check] [--send-to-mavros]
#   --send-to-mavros   把带协方差位姿输出改为 /mavros/vision_pose/pose_cov
#
# 提示：桥接默认用固定安装补偿（mounting_source:=fixed，实测安装角 roll 0.5 /
# pitch 18.8 / yaw 1.6 度，取 gravity_bridge.launch 的默认值），
# 初始化时机体保持静止即可、**无须水平**；改安装角用 gravity_bridge.launch 的
# mounting_roll_deg / mounting_pitch_deg / mounting_yaw_deg 参数。旧的
# mounting_source:=gravity 才要求初始化时机体水平。
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="${WS_DIR}/devel/setup.bash"
LOCK_FILE="/tmp/mid360s_fastlio.lock"
COMBINED_LAUNCH="/tmp/mid360s_fastlio_combined.launch"

EXPECTED_WIFI_IP="192.168.31.32"
EXPECTED_LIDAR_NET_IP="192.168.1.5"
LIDAR_DEVICE_IP="192.168.1.194"
ROS_MASTER_PORT=11311

CHECK_ONLY=0
SEND_TO_MAVROS=0
for arg in "$@"; do
    case "$arg" in
        --check)
            CHECK_ONLY=1
            ;;
        --send-to-mavros)
            SEND_TO_MAVROS=1
            ;;
        *)
            echo "用法: $0 [--check] [--send-to-mavros]" >&2
            echo "  --check            仅执行前置网络、依赖、状态及 launch 解析检查，不启动节点" >&2
            echo "  --send-to-mavros   把带协方差位姿输出改为 /mavros/vision_pose/pose_cov" >&2
            exit 2
            ;;
    esac
done

CHECK_FAILED=0

# ------------------------------------------------------------------------------
# 1. 加载 ROS 与工作空间环境
# ------------------------------------------------------------------------------
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[ERROR] ROS Noetic 环境设置文件不存在: $ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
    echo "[ERROR] 工作空间环境设置文件不存在: $WS_SETUP" >&2
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
# shellcheck source=/dev/null
source "$WS_SETUP"
set -u

export ROS_MASTER_URI="http://${EXPECTED_WIFI_IP}:${ROS_MASTER_PORT}"
export ROS_IP="${EXPECTED_WIFI_IP}"
unset ROS_HOSTNAME

# ------------------------------------------------------------------------------
# 2. 依赖检查
# ------------------------------------------------------------------------------
for cmd in roslaunch rospack rosnode rosparam ping ip flock nc; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "[ERROR] 缺少必要工具: $cmd" >&2
        if [[ $CHECK_ONLY -eq 1 ]]; then
            CHECK_FAILED=1
        else
            exit 1
        fi
    fi
done

DRIVER_PKG_DIR=$(rospack find livox_ros_driver2 2>/dev/null || true)
FASTLIO_PKG_DIR=$(rospack find fast_lio 2>/dev/null || true)
BRIDGE_PKG_DIR=$(rospack find fastlio_gravity_bridge 2>/dev/null || true)

if [[ -z "$DRIVER_PKG_DIR" ]]; then
    echo "[ERROR] 无法找到 ROS 包 livox_ros_driver2" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
fi

if [[ -z "$FASTLIO_PKG_DIR" ]]; then
    echo "[ERROR] 无法找到 ROS 包 fast_lio" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
fi

if [[ -z "$BRIDGE_PKG_DIR" ]]; then
    echo "[ERROR] 无法找到 ROS 包 fastlio_gravity_bridge" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
fi

DRIVER_LAUNCH="${DRIVER_PKG_DIR}/launch_ROS1/msg_MID360s.launch"
FASTLIO_LAUNCH="${FASTLIO_PKG_DIR}/launch/mapping_mid360s.launch"
BRIDGE_LAUNCH="${BRIDGE_PKG_DIR}/launch/gravity_bridge.launch"

if [[ ! -f "$DRIVER_LAUNCH" ]]; then
    echo "[ERROR] 驱动 launch 文件不存在: $DRIVER_LAUNCH" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
fi

if [[ ! -f "$FASTLIO_LAUNCH" ]]; then
    echo "[ERROR] 算法 launch 文件不存在: $FASTLIO_LAUNCH" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
fi

if [[ ! -f "$BRIDGE_LAUNCH" ]]; then
    echo "[ERROR] 重力对齐桥接 launch 文件不存在: $BRIDGE_LAUNCH" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
fi

[[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] ROS 环境与包依赖检查正常"

# ------------------------------------------------------------------------------
# 3. 网络与连通性检查 (带超时)
# ------------------------------------------------------------------------------
if ! ip -4 addr show | grep -qw "$EXPECTED_WIFI_IP"; then
    echo "[ERROR] 板端 Wi-Fi IP 未配置 $EXPECTED_WIFI_IP" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
else
    [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] 板端 Wi-Fi IP 检查正常 ($EXPECTED_WIFI_IP)"
fi

if ! ip -4 addr show | grep -qw "$EXPECTED_LIDAR_NET_IP"; then
    echo "[ERROR] 板端未配置雷达接口 IP $EXPECTED_LIDAR_NET_IP (enP3p49s0 可能 DOWN 或网线未连接)" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
else
    [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] 板端雷达接口 IP 检查正常 ($EXPECTED_LIDAR_NET_IP)"
fi

if ! ping -c 1 -W 2 "$LIDAR_DEVICE_IP" >/dev/null 2>&1; then
    echo "[ERROR] 雷达 $LIDAR_DEVICE_IP 连通性超时/失败 (雷达未上电或通信异常)" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
else
    [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] 雷达连通性检查正常 ($LIDAR_DEVICE_IP 可达)"
fi

# ------------------------------------------------------------------------------
# 4. 进程锁与防重复启动检查
# ------------------------------------------------------------------------------
if [[ $CHECK_ONLY -eq 1 ]]; then
    if (exec 201>"$LOCK_FILE" && flock -n 201) 2>/dev/null; then
        echo "[PASS] 进程锁状态正常 (未被占用)"
    else
        echo "[WARN] 进程锁当前已被占用 ($LOCK_FILE)，已有实例在运行"
    fi
else
    exec 200>"$LOCK_FILE"
    if ! flock -n 200; then
        echo "[ERROR] 进程锁被占用 ($LOCK_FILE)，已有 start_mid360s_fastlio 实例在运行中。" >&2
        exit 1
    fi
    echo $$ >&200
fi

# 检查系统残留进程
running_procs=()
if pgrep -f "livox_ros_driver2_node" >/dev/null 2>&1; then
    running_procs+=("livox_ros_driver2_node")
fi
if pgrep -f "fastlio_mapping" >/dev/null 2>&1; then
    running_procs+=("fastlio_mapping")
fi
if pgrep -f "gravity_bridge_node" >/dev/null 2>&1; then
    running_procs+=("gravity_bridge_node")
fi

if [[ ${#running_procs[@]} -gt 0 ]]; then
    if [[ $CHECK_ONLY -eq 1 ]]; then
        echo "[WARN] 检测到后台冲突进程在运行: ${running_procs[*]}"
    else
        echo "[ERROR] 检测到后台已有冲突进程在运行: ${running_procs[*]}，请先停止它们。" >&2
        exit 1
    fi
else
    [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] 无冲突后台进程"
fi

# 检查 ROS Master 状态与节点
if nc -z -w 1 "$EXPECTED_WIFI_IP" "$ROS_MASTER_PORT" >/dev/null 2>&1; then
    [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] 检测到运行中的 ROS Master: $ROS_MASTER_URI"
    
    sim_time=$(rosparam get /use_sim_time 2>/dev/null || echo "false")
    if [[ "$sim_time" == "true" ]]; then
        echo "[ERROR] 当前 ROS Master 处于仿真时间模式 (/use_sim_time=true)，请先关闭仿真时间或重启 ROS Master。" >&2
        if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
    else
        [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] 仿真时间模式检查通过 (/use_sim_time=false)"
    fi

    active_nodes=$(rosnode list 2>/dev/null || true)
    node_conflicts=()
    if echo "$active_nodes" | grep -qx "/livox_lidar_publisher2"; then
        node_conflicts+=("/livox_lidar_publisher2")
    fi
    if echo "$active_nodes" | grep -qx "/laserMapping"; then
        node_conflicts+=("/laserMapping")
    fi
    if echo "$active_nodes" | grep -qx "/gravity_bridge"; then
        node_conflicts+=("/gravity_bridge")
    fi

    if [[ ${#node_conflicts[@]} -gt 0 ]]; then
        echo "[ERROR] ROS Master 中已存在运行中的节点: ${node_conflicts[*]}" >&2
        if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
    else
        [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] ROS Master 中无目标节点冲突"
    fi
else
    [[ $CHECK_ONLY -eq 1 ]] && echo "[INFO] 未检测到外部 ROS Master，roslaunch 将自动创建新 Master"
fi

# ------------------------------------------------------------------------------
# 5. 生成内嵌联合 Launch 文件
# ------------------------------------------------------------------------------
cat <<'EOF' > "$COMBINED_LAUNCH"
<launch>
  <!-- 联合启动 MID-360S 驱动、FAST-LIO2 与重力对齐桥接 -->
  <arg name="send_to_mavros" default="false"/>

  <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch">
    <arg name="rviz_enable" value="false"/>
    <arg name="rosbag_enable" value="false"/>
  </include>

  <include file="$(find fast_lio)/launch/mapping_mid360s.launch">
    <arg name="rviz" value="false"/>
  </include>

  <include file="$(find fastlio_gravity_bridge)/launch/gravity_bridge.launch">
    <arg name="send_to_mavros" value="$(arg send_to_mavros)"/>
  </include>
</launch>
EOF

# ------------------------------------------------------------------------------
# 6. 解析验证 (--check) 或 前台启动
# ------------------------------------------------------------------------------
if [[ $CHECK_ONLY -eq 1 ]]; then
    echo "--- 联合 Launch 解析验证 ---"
    nodes_found=$(roslaunch --nodes "$COMBINED_LAUNCH" 2>&1)
    files_found=$(roslaunch --files "$COMBINED_LAUNCH" 2>&1)
    echo "目标节点:"
    echo "$nodes_found"
    echo "引用的 Launch 文件:"
    echo "$files_found"

    nodes_ok=1
    for expected_node in /livox_lidar_publisher2 /laserMapping /gravity_bridge; do
        if ! echo "$nodes_found" | grep -qx "$expected_node"; then
            echo "[FAIL] 联合 launch 缺少节点: $expected_node"
            nodes_ok=0
        fi
    done
    if [[ $nodes_ok -eq 1 ]]; then
        echo "[PASS] 联合 launch 节点解析正确"
    else
        echo "[FAIL] 联合 launch 节点解析异常"
        CHECK_FAILED=1
    fi

    if [[ $CHECK_FAILED -ne 0 ]]; then
        echo "=== 前置检查未通过，存在异常项 ==="
        exit 1
    fi
    echo "=== 所有前置检查项均已通过，可正常启动 ==="
    exit 0
fi

if [[ $SEND_TO_MAVROS -eq 1 ]]; then
    ROSLAUNCH_ARGS=(send_to_mavros:=true)
    echo "[INFO] 带协方差位姿输出目的地: /mavros/vision_pose/pose_cov"
else
    ROSLAUNCH_ARGS=(send_to_mavros:=false)
fi

echo "============================================================"
echo "[INFO] 启动 MID-360S 驱动与 FAST-LIO2 (前台模式)"
echo "[INFO] ROS_MASTER_URI: $ROS_MASTER_URI"
echo "[INFO] ROS_IP:         $ROS_IP"
echo "[INFO] 按 Ctrl+C 统一停止全部节点"
echo "============================================================"

exec roslaunch --screen --required "$COMBINED_LAUNCH" "${ROSLAUNCH_ARGS[@]}"
