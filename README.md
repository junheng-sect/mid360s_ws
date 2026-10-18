# MID-360S 工作空间 (mid360s_ws)

本项目为 Orange Pi 5 Max（RK3588，Ubuntu 20.04 + ROS Noetic）平台定制的 Livox MID-360S 激光雷达驱动与 FAST-LIO2 算法工作空间，采用单一独立仓库管理全部源码与内联依赖，构建不再依赖外部源码克隆。
## 本地运行 RViz

```bash
./scripts/run_rviz.sh
```

本机 Docker 显示入口及依赖已纳入本仓库，详见 [桌面 RViz 说明](docker/ros-noetic-rviz/README.md)。
沿用 host IPC 面板重绘修复，不自动最大化；与板端 `fusion.sh` 分工独立。
---

## 目录结构

```
mid360s_ws/
├── scripts/
│   ├── build.sh                   # 一键编译脚本（构建并安装本地 SDK，配置并编译 ROS 包）
│   ├── build_map.py               # 无人机建图：驱动 + FAST-LIO2 存图 + 录原始包，收尾后摆正降采样
│   ├── car_reloc.py               # M65 小车定位检查与板端重定位（经 rosbridge，不依赖 ROS master）
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
│   ├── mid360s_calib/             # 外参标定录包（通道 6 触发，原始点云 + 两路 IMU，详见包内 README）
│   │   └── launch/calib_record.launch
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
| `/fastlio/odom_world` | `nav_msgs/Odometry` | 输出（`world` → `base_link`，保留输入时间戳；`twist` 原样透传） |
| `/fastlio/vision_pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | 输出（与上面同一位姿与协方差） |
| `/tf_static` | — | `world → camera_init`（对齐旋转）与 `body → base_link`（机体安装旋转），本工作空间内唯一的发布者 |

**姿态表示机体，位置基点仍是雷达 IMU。** 输出姿态已抵消雷达 IMU 的安装倾角（本机实测约 18.8° 前倾），
水平放置时 pitch ≈ 0；初始化时机体本身倾斜时，输出会如实保留该倾角，不再被归零。位置仍是雷达
IMU 的原点，**杆臂没有补偿**，该平移偏移留给下游 / 飞控参数（例如 PX4 的 `EKF2_EV_POS_*`）处理。

TF 树：

```
world ──静态(q_a)──> camera_init ──FAST-LIO──> body ──静态(q_IB)──> base_link
                                                (雷达 IMU)        (机体，原点=IMU)
```

安装旋转 `q_IB`（`body ← base_link`，即把机体系向量转到雷达 IMU 系）由 `mounting_source` 决定：

- `fixed`（**默认**）：固定安装旋转。`mounting_roll_deg` / `mounting_pitch_deg` / `mounting_yaw_deg`
  （度）描述**雷达 IMU 在机体系（base_link，FLU）中的姿态**，按 ROS 固定轴 RPY 组合：
  `R_base←lidar = Rz(yaw)·Ry(pitch)·Rx(roll)`，节点内部取逆得到 `q_IB`。符号：pitch 正值 = 前倾
  （机头向下）；roll 正值 = 雷达右侧向下；yaw 正值 = 雷达前向朝左偏。`gravity_bridge.launch` 的默认值
  是本机实测安装角 roll `0.5` / pitch `18.8` / yaw `1.6`（10.3 台架 + 10.4 飞行标定；飞行报告的 FRD
  yaw −1.6° 对应这里的 FLU +1.6°）；节点代码默认仍是 `0/20/0`。roll、yaw 为 0 时就是纯俯仰
  `q_IB = Ry(-mounting_pitch_deg)`。它只由参数决定、
  **与初始化重力无关**，所以初始化时机体保持静止即可、**不要求水平**。
  标定结果的填法：iKalibr 以飞控 IMU（`/mavros/imu/data_raw`，MAVROS 已转 FLU）为 `ReferIMU` 时，
  Livox IMU 的 `SO3_BiToBr` 就是 `R_base←lidar`，按 ZYX 欧拉角换算成 roll/pitch/yaw 填入。
- `gravity`（兼容旧调用）：由本次初始化重力推导 `q_IB = q_a⁻¹`，前提是 **FAST-LIO 初始化时机体
  保持水平**；若初始化时机体本身倾斜，输出会把初始倾角一并当作安装角抵消，表现为读数偏小。
- `none`：不做安装补偿（输出仍是雷达 IMU 姿态，此时把 `output_child_frame:=body` 一起改回去更合适）。

非法的 `mounting_source` 或非有限的安装角（roll / pitch / yaw）会让节点明确报错并拒绝启动，不会静默兜底。

单独启动（FAST-LIO2 已在运行时）：

```bash
cd /home/orangepi/mid360s_ws && source devel/setup.bash
roslaunch fastlio_gravity_bridge gravity_bridge.launch
```

切换带协方差位姿的输出目的地到 MAVROS：

```bash
roslaunch fastlio_gravity_bridge gravity_bridge.launch send_to_mavros:=true

# 改固定安装角（launch 默认为实测值 0.5/18.8/1.6，这里只覆盖 pitch）
roslaunch fastlio_gravity_bridge gravity_bridge.launch mounting_pitch_deg:=20.0

# 填完整安装旋转（例如标定结果）
roslaunch fastlio_gravity_bridge gravity_bridge.launch \
    mounting_roll_deg:=0.35 mounting_pitch_deg:=20.12 mounting_yaw_deg:=-0.80

# 走旧的自动推导模式（要求初始化时机体水平）
roslaunch fastlio_gravity_bridge gravity_bridge.launch mounting_source:=gravity

# 关闭机体安装补偿（输出恢复为雷达 IMU 姿态，child frame 一并改回 body）
roslaunch fastlio_gravity_bridge gravity_bridge.launch \
    mounting_source:=none output_child_frame:=body
```

`send_to_mavros:=true` 时输出改为 `/mavros/vision_pose/pose_cov`，并在 `/fastlio/vision_pose_cov`
保留一路本地回显；MAVROS 侧需使用消息订阅模式（`vision_pose/tf/listen=false`），不要用 TF 监听。

行为约定（与实现一致，详见包内 `launch/gravity_bridge.launch` 注释）：

- 没收到有效初始化重力时只等待，不输出未对齐消息；latched 主题允许桥接节点晚于算法启动。
- 早于 `gravity_init` 时间戳的输入丢弃（回放/排队晚到的历史帧不属于本轮世界系，不补发、不算时间回退）。
- 姿态输出 `q_a ⊗ q_CB ⊗ q_IB`；安装旋转是常量右乘，协方差传播 `J Σ Jᵀ` 不受影响（有单测固化）。
- 非有限值、无效四元数、错误坐标系、非法 pose 协方差：拒绝发布并限频报错。
  协方差按「有限 + 对角线非负 + 对称 + 半正定」判定（相对容差 1e-9），变换后的结果同样校验。
- 重复时间戳丢弃；时间回退（已发布之后的更早帧）或运行中出现另一轮初始化：停止输出，需随 FAST-LIO 一起重启。
- 对齐旋转与协方差只做本次运行的固定变换，不额外建模初始化重力的不确定度。
- 默认 `fixed` 模式的安装旋转在启动时按参数确定，不依赖初始化重力；初始化只需保持静止
  （FAST-LIO 静态初始化），机体是否水平只表现为输出姿态里如实体现的倾角。
- 位置基点仍是雷达 IMU 原点，尚无有效速度与杆臂补偿；飞控航向策略属于后续机体接入步骤。

### 测试

```bash
cd /home/orangepi/mid360s_ws
source /opt/ros/noetic/setup.bash && source devel/setup.bash
catkin build fastlio_gravity_bridge --no-deps --catkin-make-args tests
catkin test fastlio_gravity_bridge --no-deps        # 数学 16 / 协方差 10 / 消息 22（catkin 汇总 97）
```

姿态是否真的保留（或抵消）安装倾角，可以用离线复算脚本直接从 bag 判定（不需要重跑 FAST-LIO）：

```bash
source /opt/ros/noetic/setup.bash
python3 test_artifacts/check_base_link_attitude.py bags/20260918_pitch20_world.bag
```

回放验证（隔离 Master，回放 `bags/20260917_pitch20.bag`，录制并逐项核对）：

```bash
# 注意：这两个脚本仍按旧的 gravity 模式判据（安装旋转 = q_a 的共轭），
# run_gravity_bridge_replay.sh 里已显式传 mounting_source:=gravity；要验证默认的
# fixed 模式需另加 mounting_pitch_deg 口径，留待下一轮实机/回放验收。
bash test_artifacts/run_gravity_bridge_replay.sh            # 需先确保无同名 ROS 节点在跑
source /opt/ros/noetic/setup.bash && python3 test_artifacts/analyze_gravity_bridge_replay.py
```

### 外参标定录包（mid360s_calib）

标定 MID-360S 与飞控 IMU 外参（iKalibr）时单独启动，不要与 `fusion.sh` 同时运行：

```bash
cd /home/orangepi/mid360s_ws && source devel/setup.bash
roslaunch mid360s_calib calib_record.launch
```

通道 6 拨下开始录 `/home/orangepi/test_data/bags/calib/calib_*.bag`（`/livox/lidar`、`/livox/imu`、
`/mavros/imu/data_raw`、`/mavros/timesync_status`），录包期间飞控 IMU 临时提到约 190 Hz，
关包后恢复默认并打印体检摘要。平时飞行的录包不受影响。详见 `src/mid360s_calib/README.md`。

---

## Git 仓库与协作规范

- 远程仓库: `github-mid360s:junheng-sect/mid360s_ws.git`
- 主分支: `main`
- 拉取策略: `git config pull.ff only`（强制快进合并）
- 提交与构建均在板端完成，构建产物、本地安装的 SDK、运行日志、bag 包及 PCD 文件默认被 `.gitignore` 忽略，不污染代码仓库。


## fusion 系留控制开关

板端默认启动定位、MAVROS、RC录包以及绕线器控制：

```bash
cd /home/orangepi/mid360s_ws/scripts
./fusion.sh
```

非系留测试关闭绕线器；仍保留定位、MAVROS和RC录包：

```bash
./fusion.sh --no-winch
```

也可以独立关闭录包：`./fusion.sh --no-winch --no-record`。
直接调用launch时，对应开关为 `winch:=false`（默认true）。

绕线器开启时，入口追加加载 `/home/orangepi/orange/devel/setup.bash --extend`，
保留MID360S环境；关闭时不检查绕线器包、不等待绕线器节点。
录制器为识别WinchStatus而加载消息环境，不会因此启动电机控制。
默认使用 `/home/orangepi/orange/src/tether_winch_control/config/winch.yaml`，
本次接入不修改其中的DUTY或自动启用参数。启动会先发送备用值，随后按配置自动启用。
请勿同时运行独立的绕线器launch或本机DUTY测试脚本；本入口检测板端重复节点，
无法检测其他电脑上的发送程序。`--no-winch` 不会替你停止另一个独立发送程序。

Ctrl+C会统一结束本次fusion启动的节点，绕线器正常退出尝试发送YAML的fallback_duty，
不会套用本机脉冲脚本Ctrl+C发送300的规则；网络断开时不能保证备用指令送达。
沿用原有 `roslaunch --required` 规则：绕线器进程异常退出会结束本次整条链路；
绕线器报告FAULT但进程仍在时不会触发整链退出。


## 小车定位（car_reloc.py）

M65 小车（spirit，`192.168.31.231`，rosbridge `9090`）开机时直接读回关机前的位姿，不做搜索：
关机时挪过车，定位就是错的，App 却照样报定位正常。`scripts/car_reloc.py` 在板端检查车的定位，
不合格时拿车的 `/scan` 在它的 `/map` 上做全局匹配（不需要初值），调 `/set_relocation` 重定位，
再复核一遍。只经 rosbridge 和车通信，不依赖板端 ROS master，fusion 没起也能用；全程不让车动。

```bash
cd /home/orangepi/mid360s_ws/scripts
./car_reloc.py                     # 检查，不合格才重定位
./car_reloc.py --check             # 只检查，不发
./car_reloc.py --force             # 检查合格也重发一次
./car_reloc.py --wait-online 180   # 车刚开机：最多等 180 s 上线再做
```

判据（板端激光得分、局部再匹配偏移、车自报 `/local_global_score[1]` 和 `/submap_in_map_score`）
和退出码见脚本开头说明。依赖 roslibpy（`pip --user`）、numpy、`python3-opencv`，板端已装。
室内地图上一次完整重定位约 16 s：检查约 2 s，全局匹配约 4 s，车端搜索恢复约 4 s。

## 无人机建图（build_map.py）

`scripts/build_map.py` 拉起 MID-360S 驱动、FAST-LIO2 和重力对齐桥接，同时录原始包；按 Ctrl+C 结束后，
FAST-LIO 写出地图，脚本再把地图摆正、降采样、出预览图。每次的产出放在 `~/test_data/mapping/<时间>/`。
FAST-LIO 的 `pcd_save_en` 只在脚本生成的 launch 里打开，`mid360s.yaml` 不动，fusion 飞行不受影响。

```bash
cd /home/orangepi/mid360s_ws/scripts
./build_map.py                    # 实机建图：等提示「初始化完成」再走，回到起点按 Ctrl+C
./build_map.py --name lab         # 输出目录名加后缀
./build_map.py --bag ~/test_data/mapping/<时间>/raw.bag   # 用录好的原始包离线重建
./build_map.py --post ~/test_data/mapping/<时间>          # 只重做后处理（比如换 --voxel）
```

| 文件 | 内容 |
|---|---|
| `raw.bag` | 实机：`/livox/lidar` `/livox/imu` `/tf_static` `/fastlio/gravity_init` `/Odometry`（离线时是只含后三个话题的 `odom.bag`） |
| `scans.pcd` | FAST-LIO 原样输出：`camera_init` 系、稠密、未降采样 |
| `map_world.pcd` | 摆正到 `world`（z 朝上，原点 = 起点时的雷达 IMU，yaw 不动），0.05 m 体素，字段 `x y z intensity` |
| `preview.png` | 俯视、侧视和轨迹 |
| `meta.json` | 点数、`world←camera_init`、地面拟合（起点离地高度、残余倾斜）、首尾位姿差、git 版本 |

注意：
- FAST-LIO 只在节点正常退出时一次写出 `scans.pcd`。要用 Ctrl+C（或 SIGTERM）结束，脚本会按顺序收尾；
  ssh 断开（SIGHUP）也按正常收尾存图。kill -9 或者掉电，这一次的地图就没了，原始包可能也不完整。
- 每分钟约 400 万原始点、约 130 MB；单次建图建议控制在 5 min 内。
- 离线重建不用仿真时间：FAST-LIO 只认消息时间戳，而它的主循环在仿真时间下，回放一停就出不来，地图存不下来。
- 前置检查和 `start_mid360s_fastlio.sh` 一样，但要求板端没有正在运行的 ROS master。实机还会先等 chrony 对上时间（同 fusion.sh，最多 60 s）：雷达用板端系统时间打戳，建图中时间一跳地图就作废；没外网时加 `--allow-unsynced-clock`。

