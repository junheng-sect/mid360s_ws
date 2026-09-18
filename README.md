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
│   ├── fastlio_gravity_bridge/    # 重力对齐桥接包（world 输出 + 带协方差位姿）
│   │   ├── launch/gravity_bridge.launch
│   │   ├── include/fastlio_gravity_bridge/   # 对齐旋转与协方差变换（纯数学 + 节点类）
│   │   └── test/                  # 数学 / 协方差 / 消息测试
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

使用一键启动脚本管理驱动、算法与重力对齐桥接节点：

1. **仅前置诊断（不启动节点）**:
   ```bash
   ./scripts/start_mid360s_fastlio.sh --check
   ```
2. **前台联合启动**:
   ```bash
   ./scripts/start_mid360s_fastlio.sh
   ```
   按 `Ctrl+C` 可同时安全停止驱动、算法与桥接节点，避免残留后台孤儿进程。
3. **联合启动并把带协方差位姿发给 MAVROS**:
   ```bash
   ./scripts/start_mid360s_fastlio.sh --send-to-mavros
   ```

### 重力对齐桥接（fastlio_gravity_bridge）

桥接节点订阅 FAST-LIO2 的输出，把 `camera_init` 下的里程计转换到重力对齐的 `world`（+Z 向上，
原点与 `camera_init` 相同，只做最小旋转、不调整 yaw，不代表已对准地理东/北）：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | 输入（`camera_init` → `body`） |
| `/fastlio/gravity_init` | `geometry_msgs/Vector3Stamped`，latched | 输入：本次初始化的重力（`camera_init` 下），由 FAST-LIO2 在 IMU 初始化完成时发布一次 |
| `/fastlio/odom_world` | `nav_msgs/Odometry` | 输出（`world` → `body`，保留输入时间戳；`twist` 原样透传） |
| `/fastlio/vision_pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | 输出（与上面同一位姿与协方差） |
| `/tf_static` | — | `world → camera_init`，本次运行的固定对齐旋转，本工作空间内唯一发布者 |

单独启动（FAST-LIO2 已在运行时）：

```bash
cd /home/orangepi/mid360s_ws && source devel/setup.bash
roslaunch fastlio_gravity_bridge gravity_bridge.launch
```

切换带协方差位姿的输出目的地到 MAVROS：

```bash
roslaunch fastlio_gravity_bridge gravity_bridge.launch send_to_mavros:=true
```

`send_to_mavros:=true` 时输出改为 `/mavros/vision_pose/pose_cov`，并在 `/fastlio/vision_pose_cov`
保留一路本地回显；MAVROS 侧需使用消息订阅模式（`vision_pose/tf/listen=false`），不要用 TF 监听。

行为约定（与实现一致，详见包内 `launch/gravity_bridge.launch` 注释）：

- 没收到有效初始化重力时只等待，不输出未对齐消息；latched 主题允许桥接节点晚于算法启动。
- 早于 `gravity_init` 时间戳的输入丢弃（回放/排队晚到的历史帧不属于本轮世界系，不补发、不算时间回退）。
- 非有限值、无效四元数、错误坐标系、非法 pose 协方差：拒绝发布并限频报错。
  协方差按「有限 + 对角线非负 + 对称 + 半正定」判定（相对容差 1e-9），变换后的结果同样校验。
- 重复时间戳丢弃；时间回退（已发布之后的更早帧）或运行中出现另一轮初始化：停止输出，需随 FAST-LIO 一起重启。
- 对齐旋转与协方差只做本次运行的固定变换，不额外建模初始化重力的不确定度。
- 本轮输出代表雷达 IMU 位姿，尚无有效速度；安装旋转与飞控航向策略属于后续机体接入步骤。

### 测试

```bash
cd /home/orangepi/mid360s_ws
source /opt/ros/noetic/setup.bash && source devel/setup.bash
catkin build fastlio_gravity_bridge --no-deps --catkin-make-args tests
catkin test fastlio_gravity_bridge --no-deps        # 47 个用例：数学 / 协方差 / 消息
```

回放验证（隔离 Master，回放 `bags/20260917_pitch20.bag`，录制并逐项核对）：

```bash
bash test_artifacts/run_gravity_bridge_replay.sh            # 需先确保无同名 ROS 节点在跑
source /opt/ros/noetic/setup.bash && python3 test_artifacts/analyze_gravity_bridge_replay.py
```

---

## Git 仓库与协作规范

- 远程仓库: `github-mid360s:junheng-sect/mid360s_ws.git`
- 主分支: `main`
- 拉取策略: `git config pull.ff only`（强制快进合并）
- 提交与构建均在板端完成，构建产物、本地安装的 SDK、运行日志、bag 包及 PCD 文件默认被 `.gitignore` 忽略，不污染代码仓库。
