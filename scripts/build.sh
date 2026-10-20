#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# MID-360S 工作空间一键构建脚本
# 1. 自动定位工作空间根目录
# 2. 显式加载 ROS Noetic 环境
# 3. 编译仓库内 third_party/Livox-SDK2 并安装至工作空间 .deps/livox-sdk2
# 4. 编译仓库内 third_party/gtsam-4.0.3（Voxel-SLAM 依赖）并安装至 .deps/gtsam-4.0.3
#    首次约 15 min，之后增量；并行数用环境变量 GTSAM_JOBS（默认 6，模板展开吃内存）
# 5. 配置 catkin 参数 (ROS1, Python 3, Release, LIVOX_SDK_ROOT, GTSAM_DIR) 并执行 catkin build
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
echo "[INFO] 步骤 1/3: 构建与本地安装 Livox-SDK2"
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

GTSAM_SRC_DIR="${WS_DIR}/third_party/gtsam-4.0.3"
GTSAM_BUILD_DIR="${WS_DIR}/.deps/build_gtsam"
GTSAM_INSTALL_DIR="${WS_DIR}/.deps/gtsam-4.0.3"
if [[ ! -f "${GTSAM_SRC_DIR}/CMakeLists.txt" ]]; then
    echo "[ERROR] 仓库内 GTSAM 源码目录不存在: $GTSAM_SRC_DIR" >&2
    exit 1
fi

echo "============================================================"
echo "[INFO] 步骤 2/3: 构建与本地安装 GTSAM 4.0.3"
echo "[INFO] 源码目录: $GTSAM_SRC_DIR"
echo "[INFO] 构建目录: $GTSAM_BUILD_DIR"
echo "[INFO] 安装前缀: $GTSAM_INSTALL_DIR"
echo "============================================================"

# 选项与 Voxel-SLAM README 的要求一致：用系统 Eigen（仓库里没带 GTSAM 自带的 Eigen）、不开 march=native、
# 不编测试/例子/unstable/TBB。INSTALL_RPATH 让 libgtsam.so 找得到同目录的 libmetis-gtsam.so，运行时不用设 LD_LIBRARY_PATH
cmake -B "$GTSAM_BUILD_DIR" -S "$GTSAM_SRC_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$GTSAM_INSTALL_DIR" \
    -DCMAKE_INSTALL_RPATH="${GTSAM_INSTALL_DIR}/lib" \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_DOCS=OFF \
    -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=OFF

cmake --build "$GTSAM_BUILD_DIR" --target install -j"${GTSAM_JOBS:-6}"

if [[ ! -f "${GTSAM_INSTALL_DIR}/lib/libgtsam.so.4" || ! -f "${GTSAM_INSTALL_DIR}/lib/cmake/GTSAM/GTSAMConfig.cmake" ]]; then
    echo "[ERROR] GTSAM 安装验证失败，未找到库或 CMake 配置: $GTSAM_INSTALL_DIR" >&2
    exit 1
fi
echo "[INFO] GTSAM 本地安装验证通过。"

echo "============================================================"
echo "[INFO] 步骤 3/3: 执行 catkin build"
echo "[INFO] 工作空间: $WS_DIR"
echo "============================================================"

cd "$WS_DIR"

catkin config \
    --extend /opt/ros/noetic \
    --cmake-args \
        -DROS_EDITION=ROS1 \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DCMAKE_BUILD_TYPE=Release \
        -DLIVOX_SDK_ROOT="$SDK_INSTALL_DIR" \
        -DGTSAM_DIR="${GTSAM_INSTALL_DIR}/lib/cmake/GTSAM"

catkin build "$@"

echo "============================================================"
echo "[INFO] MID-360S 工作空间构建成功！"
echo "[INFO] 生效环境请执行: source ${WS_DIR}/devel/setup.bash"
echo "============================================================"
