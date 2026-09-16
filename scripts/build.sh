#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# MID-360S 工作空间一键构建脚本
# 1. 自动定位工作空间根目录
# 2. 显式加载 ROS Noetic 环境
# 3. 编译仓库内 third_party/Livox-SDK2 并安装至工作空间 .deps/livox-sdk2
# 4. 配置 catkin 参数 (ROS1, Python 3, Release, LIVOX_SDK_ROOT) 并执行 catkin build
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_SETUP="/opt/ros/noetic/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[ERROR] ROS Noetic 环境设置文件不存在: $ROS_SETUP" >&2
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
set -u

SDK_SRC_DIR="${WS_DIR}/third_party/Livox-SDK2"
if [[ ! -d "$SDK_SRC_DIR" ]]; then
    echo "[ERROR] 仓库内 Livox-SDK2 源码目录不存在: $SDK_SRC_DIR" >&2
    exit 1
fi

SDK_BUILD_DIR="${WS_DIR}/.deps/build_livox_sdk2"
SDK_INSTALL_DIR="${WS_DIR}/.deps/livox-sdk2"

echo "============================================================"
echo "[INFO] 步骤 1/2: 构建与本地安装 Livox-SDK2"
echo "[INFO] 源码目录: $SDK_SRC_DIR"
echo "[INFO] 构建目录: $SDK_BUILD_DIR"
echo "[INFO] 安装前缀: $SDK_INSTALL_DIR"
echo "============================================================"

mkdir -p "$SDK_BUILD_DIR"
mkdir -p "$SDK_INSTALL_DIR"

cmake -B "$SDK_BUILD_DIR" -S "$SDK_SRC_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$SDK_INSTALL_DIR"

cmake --build "$SDK_BUILD_DIR" --target install -j"$(nproc)"

if [[ ! -f "${SDK_INSTALL_DIR}/lib/liblivox_lidar_sdk_static.a" || ! -f "${SDK_INSTALL_DIR}/include/livox_lidar_api.h" ]]; then
    echo "[ERROR] Livox-SDK2 安装验证失败，未找到必要的静态库或头文件: $SDK_INSTALL_DIR" >&2
    exit 1
fi
echo "[INFO] Livox-SDK2 本地安装验证通过。"

echo "============================================================"
echo "[INFO] 步骤 2/2: 执行 catkin build"
echo "[INFO] 工作空间: $WS_DIR"
echo "============================================================"

cd "$WS_DIR"

catkin config \
    --extend /opt/ros/noetic \
    --cmake-args \
        -DROS_EDITION=ROS1 \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DCMAKE_BUILD_TYPE=Release \
        -DLIVOX_SDK_ROOT="$SDK_INSTALL_DIR"

catkin build "$@"

echo "============================================================"
echo "[INFO] MID-360S 工作空间构建成功！"
echo "[INFO] 生效环境请执行: source ${WS_DIR}/devel/setup.bash"
echo "============================================================"
