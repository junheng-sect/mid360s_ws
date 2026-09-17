# MID-360S 工作空间 (mid360s_ws)

本项目为 Orange Pi 5 Max（RK3588，Ubuntu 20.04 + ROS Noetic）平台定制的 Livox MID-360S 激光雷达驱动与 FAST-LIO2 算法工作空间，采用单一独立仓库管理全部源码与内联依赖，构建不再依赖外部源码克隆。
## 本地运行rviz
/home/zjh/docker/ros-noetic-rviz/scripts/run_rviz.sh
---

## 目录结构

```
mid360s_ws/
├── scripts/
│   ├── build.sh                   # 一键编译脚本（构建并安装本地 SDK，配置并编译 ROS 包）
│   └── start_mid360s_fastlio.sh   # 一键启动驱动与 FAST-LIO2 算法节点（含前置检查与进程锁）
├── src/
│   ├── FAST_LIO/                  # FAST-LIO2 算法包
│   │   ├── config/mid360s.yaml    # MID-360S 传感器配置参数
│   │   ├── include/ikd-Tree/      # ikd-Tree 数据结构源码（以普通目录纳入）
│   │   └── launch/mapping_mid360s.launch
│   └── livox_ros_driver2/         # Livox ROS 驱动（已适配 MID-360S 与本地 SDK 路径）
│       ├── config/MID360s_config.json
│       ├── launch_ROS1/msg_MID360s.launch
│       └── package.xml            # ROS1 package.xml（版本受控跟踪）
└── third_party/
    └── Livox-SDK2/                # Livox SDK 2.0 源码（完全内嵌，避免拉取官方仓库）
```

---

## 第三方组件来源与适配说明

本仓库完整收纳以下四个上游开源组件，移除嵌套 Git 元数据与子模块配置，作为独立源码跟踪：

### 1. FAST_LIO
- **上游地址**: `https://github.com/hku-mars/FAST_LIO.git`
- **基线提交**: `7cc4175` ("Support MARSIM simulator.")
- **许可证**: GPLv2 (见 `src/FAST_LIO/LICENSE`)
- **已有适配**:
  - `CMakeLists.txt` 与 `package.xml`: 依赖项由 `livox_ros_driver` 调整为 `livox_ros_driver2`，引入 `eigen_conversions`。
  - `src/laserMapping.cpp`, `src/preprocess.cpp`, `src/preprocess.h`: 消息回调与预处理接口适配 `livox_ros_driver2::CustomMsg` 自定义点云消息。
  - `config/mid360s.yaml`: 针对 MID-360S 雷达与板载 IMU 优化的外参、噪声参数与点云特征配置。
  - `launch/mapping_mid360s.launch`: MID-360S 专用建图启动文件。

### 2. ikd-Tree
- **上游地址**: `https://github.com/hku-mars/ikd-Tree.git`
- **基线提交**: `e2e3f4e` ("Fix bugs in max_dist of nearest search")
- **许可证**: GPLv2
- **已有适配**: 将原 `FAST_LIO` 下的 Git 子模块（gitlink 160000）解除，源码直接收纳在 `src/FAST_LIO/include/ikd-Tree`，保持原有 C++ 包含路径不变。

### 3. livox_ros_driver2
- **上游地址**: `https://github.com/Livox-SDK/livox_ros_driver2.git`
- **基线提交**: `4a1def9` ("Support Avia2 Lidar, fix the issue that no point cloud is published when all config items are omitted.")
- **许可证**: BSD 3-Clause (见 `src/livox_ros_driver2/LICENSE.txt`)
- **已有适配**:
  - `config/MID360s_config.json`: 雷达 IP (`192.168.1.194`) 与通信端口配置。
  - `config/display_point_cloud_ROS1.rviz`: RViz 点云展示视图配置。
  - `CMakeLists.txt`: 引入 `LIVOX_SDK_ROOT` 变量支持，严格指定本地编译安装的 SDK 头文件与静态库路径，未找到时清晰报错，防止串用系统全局版本。
  - `.gitignore`: 移除 `package.xml` 忽略规则，显式受控跟踪 ROS1 部署清单。

### 4. Livox-SDK2
- **上游地址**: `https://github.com/Livox-SDK/Livox-SDK2.git`
- **基线提交**: `08f523c` ("Support Avia2 Lidar, set fov/echo/ITO/fog filter mode and NTP server ip, add ip_set and info_get samples.")
- **许可证**: Apache-2.0 (见 `third_party/Livox-SDK2/LICENSE.txt`)
- **已有适配**: 整体收入 `third_party/Livox-SDK2`，由构建脚本自动编译并安装到工作空间临时依赖目录 `.deps/livox-sdk2`。

---

## 系统依赖要求

- 操作系统: Ubuntu 20.04 LTS (aarch64 / RK3588)
- ROS 版本: ROS Noetic (`desktop-full`)
- 构建工具: `catkin-tools`, `cmake (>= 3.0)`, `make`, `gcc/g++ (>= 9.4)`
- 库依赖: PCL (>= 1.10), Eigen3, Boost (>= 1.54), `libapr1-dev`

---

## 构建步骤

工作空间构建采用提供的 `scripts/build.sh` 脚本统一执行，过程包含本地 SDK 构建与 ROS 包编译：

```bash
cd /home/orangepi/mid360s_ws
./scripts/build.sh
```

构建完成后加载环境：

```bash
source devel/setup.bash
```

---

## 运行与启动

使用一键启动脚本管理驱动与算法节点：

1. **仅前置诊断（不启动节点）**:
   ```bash
   ./scripts/start_mid360s_fastlio.sh --check
   ```
2. **前台联合启动**:
   ```bash
   ./scripts/start_mid360s_fastlio.sh
   ```
   按 `Ctrl+C` 可同时安全停止驱动与算法节点，避免残留后台孤儿进程。

---

## Git 仓库与协作规范

- 远程仓库: `github-mid360s:junheng-sect/mid360s_ws.git`
- 主分支: `main`
- 拉取策略: `git config pull.ff only`（强制快进合并）
- 提交与构建均在板端完成，构建产物、本地安装的 SDK、运行日志、bag 包及 PCD 文件默认被 `.gitignore` 忽略，不污染代码仓库。
