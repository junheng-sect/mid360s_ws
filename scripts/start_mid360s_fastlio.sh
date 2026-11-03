#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# 一键前台启动 Livox MID-360S 驱动、FAST-LIO2 与重力对齐桥接
# 路径: /home/orangepi/mid360s_ws/scripts/start_mid360s_fastlio.sh
# 用法: ./start_mid360s_fastlio.sh [--check] [--send-to-mavros] [--ev-mode pose|odom]
#   --send-to-mavros   把外部定位送给飞控：pose 方式送 /mavros/vision_pose/pose_cov，
#                      odom 方式送 /mavros/odometry/out（位姿 + 速度，要 MAVROS 用 MAVLink2）
#   --ev-mode MODE     重力桥 ev_mode，默认 pose；说明见 gravity_bridge.launch
#
# 提示：桥接默认用固定安装补偿（mounting_source:=fixed，实测安装角 roll 0.5 /
# pitch 18.8 / yaw 1.6 度，取 gravity_bridge.launch 的默认值），
# 初始化时机体保持静止即可、**无须水平**；改安装角用 gravity_bridge.launch 的
# mounting_roll_deg / mounting_pitch_deg / mounting_yaw_deg 参数。旧的
# mounting_source:=gravity 才要求初始化时机体水平。
#
# 运行期间同 fusion.sh 冻结板端时钟（clock_guard.sh）：起 roslaunch 前停 chronyd，退出后收拢
# 残留节点、全退了再启动 chronyd。没外网也能启动，运行中联网不会跳时间。
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="${WS_DIR}/devel/setup.bash"
LOCK_FILE="/tmp/mid360s_fastlio.lock"
COMBINED_LAUNCH="/tmp/mid360s_fastlio_combined.launch"

EXPECTED_LIDAR_NET_IP="192.168.1.5"
LIDAR_DEVICE_IP="192.168.1.194"
ROS_MASTER_PORT=11311

CHECK_ONLY=0
SEND_TO_MAVROS=0
EV_MODE="pose"
usage() {
    echo "用法: $0 [--check] [--send-to-mavros] [--ev-mode pose|odom]" >&2
    echo "  --check            仅执行前置网络、依赖、状态及 launch 解析检查，不启动节点" >&2
    echo "  --send-to-mavros   把外部定位送给飞控（pose 方式 /mavros/vision_pose/pose_cov，odom 方式 /mavros/odometry/out）" >&2
    echo "  --ev-mode MODE     pose（默认，只发位姿）/ odom（位姿 + 速度）" >&2
    exit 2
}
while [[ $# -gt 0 ]]; do
    case "$1" in
        --check)
            CHECK_ONLY=1
            ;;
        --send-to-mavros)
            SEND_TO_MAVROS=1
            ;;
        --ev-mode)
            EV_MODE="${2:-}"
            shift
            ;;
        *)
            usage
            ;;
    esac
    shift
done
case "$EV_MODE" in pose|odom) ;; *) echo "--ev-mode 只能是 pose 或 odom（收到：$EV_MODE）" >&2; usage ;; esac

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

# ROS 核心用板内固定地址（ros0 10.250.32.1，master 走 127.0.0.1），不依赖 Wi-Fi；地址不在时第 3 步报错。
# WIFI_IP 只用于提示本机 RViz 怎么连，没连 Wi-Fi 为空
if source "${SCRIPT_DIR}/ros_net.sh"; then CORE_NET_OK=1; else CORE_NET_OK=0; fi
WIFI_IP="${ROS_NET_WIFI_IP}"
# 运行期间冻结板端时钟，见文件头
source "${SCRIPT_DIR}/clock_guard.sh"

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
if [[ $CORE_NET_OK -ne 1 ]]; then
    echo "[ERROR] 板内固定地址 ${ROS_CORE_IP} 不在（nmcli con up ros-core）" >&2
    if [[ $CHECK_ONLY -eq 1 ]]; then CHECK_FAILED=1; else exit 1; fi
else
    [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] ROS 核心地址正常 ($ROS_IP，master 走 127.0.0.1)"
fi
if [[ -z "$WIFI_IP" ]]; then
    echo "[WARN] Wi-Fi 未连接：核心照常运行，只是本机 RViz / ssh 暂时连不上" >&2
else
    [[ $CHECK_ONLY -eq 1 ]] && echo "[PASS] 板端 Wi-Fi IP $WIFI_IP（本机 RViz 经它访问 $ROS_IP）"
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
if nc -z -w 1 127.0.0.1 "$ROS_MASTER_PORT" >/dev/null 2>&1; then
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
  <arg name="ev_mode" default="pose"/>

  <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch">
    <arg name="rviz_enable" value="false"/>
    <arg name="rosbag_enable" value="false"/>
  </include>

  <include file="$(find fast_lio)/launch/mapping_mid360s.launch">
    <arg name="rviz" value="false"/>
  </include>

  <include file="$(find fastlio_gravity_bridge)/launch/gravity_bridge.launch">
    <arg name="send_to_mavros" value="$(arg send_to_mavros)"/>
    <arg name="ev_mode" value="$(arg ev_mode)"/>
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
    ROSLAUNCH_ARGS=(send_to_mavros:=true "ev_mode:=$EV_MODE")
    if [[ $EV_MODE == odom ]]; then
        echo "[INFO] 外部定位（odom 方式，位姿 + 速度）送: /mavros/odometry/out"
    else
        echo "[INFO] 带协方差位姿输出目的地: /mavros/vision_pose/pose_cov"
    fi
else
    ROSLAUNCH_ARGS=(send_to_mavros:=false "ev_mode:=$EV_MODE")
fi

echo "============================================================"
echo "[INFO] 启动 MID-360S 驱动与 FAST-LIO2 (前台模式)"
echo "[INFO] ROS_MASTER_URI: $ROS_MASTER_URI"
echo "[INFO] ROS_IP:         $ROS_IP"
echo "[INFO] 按 Ctrl+C 统一停止全部节点"
echo "============================================================"

# 冻结板端时钟：chronyd 没在跑就先拉起来看一眼对没对上（不等），再停掉；冻结不了就不启动
clock_ensure_running || true
if timeout 3 chronyc -n tracking 2>/dev/null | grep -q "^Leap status *: Not synchronised"; then
    echo "[WARN] 板端 UTC 时间未同步：按板端本地时间运行（运行中时钟冻结、不会跳，退出后再对时）"
fi
if ! clock_freeze; then
    echo "[ERROR] 冻结不了板端时钟（另一个定位核心实例在跑，或 /etc/sudoers.d/mid360s-chrony 没装），运行中可能跳时间，不启动" >&2
    exit 1
fi
echo "[INFO] 已停 chronyd：运行中系统时钟不跳、不调速，退出后恢复对时"

# roslaunch 作为子进程跑（不再 exec），这样退出后还能收拢孤儿节点、恢复 chronyd。
# Ctrl+C / SIGTERM 转发给 roslaunch；终端 Ctrl+C 时它自己也会收到一份，重复 SIGINT 不影响收尾。
set +e
# shellcheck disable=SC2046
env $(clock_env fastlio) roslaunch --screen --required "$COMBINED_LAUNCH" "${ROSLAUNCH_ARGS[@]}" &
RL_PID=$!
trap 'kill -INT "$RL_PID" 2>/dev/null' INT TERM
while :; do
    wait "$RL_PID"; RL_RC=$?
    kill -0 "$RL_PID" 2>/dev/null || break
done
trap - INT TERM
reap_tagged fastlio "雷达/FAST-LIO/桥接"
clock_release; _rc=$?
if [[ $_rc -eq 0 ]]; then
    echo "[INFO] chronyd 已恢复（时间若有大偏差在此时一次校正）"
elif [[ $_rc -eq 2 ]]; then
    echo "[WARN] 还有残留进程（pid $(tagged_pids | tr '\n' ' ')），chronyd 保持停止；处理掉后执行 sudo systemctl start chrony" >&2
else
    echo "[WARN] chronyd 没起来，处理残留后执行 sudo systemctl start chrony" >&2
fi
exit "$RL_RC"
