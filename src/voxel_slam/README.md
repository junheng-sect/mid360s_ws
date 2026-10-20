# voxel_slam（Voxel-SLAM，本仓库内嵌版）

hku-mars 的激光-惯性 SLAM（[论文 arXiv:2410.08935](https://arxiv.org/abs/2410.08935)，[视频](https://youtu.be/Cg9W01aIUzE)），
ICRA HILTI 2023 激光单会话第 2、ICCV 2023 激光-惯性赛道第 1。一共五个模块：

- **初始化**：静止或运动中都能初始化，给出初始状态和局部地图；
- **里程计**：估计当前状态，并检测系统是否发散；
- **局部建图**：在滑动窗口里做激光-惯性 BA，修正状态和局部地图；
- **回环**：可以在多次建图（多会话）之间找回环；
- **全局建图**：收尾时做分层全局 BA，把整张图拉一致。

**在本工作空间里只用来离线建图**：它不发布 `nav_msgs/Odometry` 和 TF，飞行定位仍是 FAST-LIO。
日常建图用 [`drone_mapping`](../drone_mapping/README.md) 包，它会自动起本节点、回放原始包、触发全局 BA，并拼出 `map_world.pcd`。
10-19/20 用同样三个原始包对比：闭环差 2 mm（FAST-LIO 8–11 cm），面厚度约 6 mm（FAST-LIO 约 11 mm）。

- 来源：上游 `https://github.com/hku-mars/Voxel-SLAM` 提交 `70fc8a2`（2025-04-07），只收了 `VoxelSLAM/` 子目录，许可证 GPLv2（见 `LICENSE`）。
- 改动：依赖和 `CustomMsg` 回调由 `livox_ros_driver` 改成 `livox_ros_driver2`（`CMakeLists.txt`、`package.xml`、
  `src/feature_point.hpp`、`src/voxelslam.cpp`，共 7 行），其余没动。
- 依赖：GTSAM 4.0.3（仓库内 `third_party/gtsam-4.0.3`），PCL、Eigen 用系统自带的。

## 编译

```bash
cd /home/orangepi/mid360s_ws
./scripts/build.sh                 # 先编 GTSAM 到 .deps/gtsam-4.0.3，再 catkin build 全部包
catkin build voxel_slam            # 之后单独重编本包（build.sh 跑过一次以后，catkin 参数里已经有 GTSAM_DIR）
```

## 用法

### 1. 推荐：通过 drone_mapping 建图

```bash
source /opt/ros/noetic/setup.bash && source /home/orangepi/mid360s_ws/devel/setup.bash
rosrun drone_mapping build_map                    # 实机：录原始包，Ctrl+C 后自动离线跑 Voxel-SLAM 并拼图
rosrun drone_mapping build_map --bag RAW.bag      # 用录好的原始包离线重建
```

细节见 [drone_mapping/README.md](../drone_mapping/README.md)。

### 2. 手动离线跑一个原始包

`drone_mapping/launch/voxelslam.launch` 会加载 `drone_mapping/config/voxelslam_mid360s.yaml`（上游 `mid360.yaml` 的副本），
并把 `save_path`、`bagname`、`is_save_map=1` 改成命令行给的值。开三个终端，每个终端先 source：

```bash
roscore                                                                   # 终端 1
```

```bash
mkdir -p /tmp/vs_out && roslaunch drone_mapping voxelslam.launch save_path:=/tmp/vs_out/   # 终端 2
```

```bash
rosbag play RAW.bag --topics /livox/lidar /livox/imu                     # 终端 3
```

放完以后，等终端 2 不再输出（也就是处理完了），再触发全局 BA：

```bash
rosparam set /finish true
```

位姿写到 `/tmp/vs_out/vs/alidarState.txt`（房间里几秒），文件不再变大以后 Ctrl+C 停掉终端 2。`save_path` 要以 `/` 结尾；`<save_path><bagname>/`（默认 `vs/`）
如果事先存在，节点会直接退出。这样只能拿到位姿和逐帧点云；要得到 `map_world.pcd`，还是用 `drone_mapping`。

### 3. 带 RViz 实时看

```bash
roslaunch voxel_slam vxlm_mid360.launch           # 上游 launch：加载 config/mid360.yaml，并打开 rviz_cfg/back.rviz
roslaunch voxel_slam vxlm_mid360.launch rviz:=false
```

上游 `mid360.yaml` 里 `is_save_map: 0`，什么都不存，只适合看效果。RViz 要在板端桌面上开（NoMachine，ssh 下设 `DISPLAY=:0`）；
本机看要把 RViz 加到 Voxel-SLAM 的点云话题上。实时跑时板端约占 1 个核，可以和 FAST-LIO 同时跑。

## 输入和输出

| 方向 | 内容 |
|---|---|
| 订阅 | `/livox/lidar`（`livox_ros_driver2/CustomMsg`，雷达 1000 帧队列），`/livox/imu`（`sensor_msgs/Imu`，80000 条队列） |
| 发布 | 只有 `sensor_msgs/PointCloud2`：`/map_scan`、`/map_cmap`、`/map_pmap`、`/map_init`、`/map_path`、`/map_true`、`/map_test`（RViz 配置里都有） |
| 参数 | 都在全局命名空间：`General/…`、`Odometry/…`、`LocalBA/…`、`Loop/…`、`GBA/…`，以及收尾开关 `/finish` |
| 文件（`is_save_map: 1`） | `<save_path><bagname>/<序号>.pcd`：逐帧点云（按 `down_size` 体素降采样过）；`<save_path><bagname>/alidarState.txt`：位姿；`<save_path>edge.txt`：多会话回环边 |

`alidarState.txt` 每行一帧，共 26 列：
- 第 1–8 列：`t x y z qx qy qz qw`，是 IMU 在扫描结束时刻的位姿，世界系已按它自己估的重力摆正，原点是起点；
- 第 9–11 列：速度；
- 第 12–14 列：陀螺零偏；
- 第 15–17 列：加速度计零偏；
- 第 18–20 列：重力；
- 第 21–26 列：6 个附加量。

`drone_mapping` 只用前 8 列和重力那 3 列。

## 参数（`config/mid360.yaml`）

| 参数 | 值 | 含义 |
|---|---|---|
| `General/blind` | 0.5 | 离雷达这么近（m）的点不用 |
| `General/point_filter_num` | 3 | 每 3 个点取 1 个 |
| `General/extrinsic_tran` / `extrinsic_rota` | `[-0.011, -0.02329, 0.04412]` / 单位阵 | 雷达 → IMU 外参，和 `FAST_LIO/config/mid360s.yaml` 一致 |
| `General/previous_map` | 空 | 多会话时要加载的旧会话，格式 `名字: 阈值, …` |
| `Odometry/down_size` | 0.1 | 每帧点云的降采样体素（m） |
| `Odometry/voxel_size` | 1 | 里程计体素地图的体素（m） |
| `Odometry/max_init` | 30 | 初始化用的帧数 |
| `LocalBA/win_size` / `thread_num` | 10 / 5 | 滑窗大小、局部 BA 线程数 |
| `Loop/jud_default` | 0.45 | 回环判定阈值 |
| `GBA/voxel_size` / `total_max_iter` | 2 / 6 | 全局 BA 的体素（m）、迭代次数 |

其他配置是上游给别的雷达和场景用的。比如 MARS 飞行数据用的 `avia_fly.yaml` 更粗：`down_size` 0.5、`voxel_size` 4、
`GBA/voxel_size` 15，并开了 `Loop/isHighFly`。室外大场景我们还没测过，要改参数先拿原始包离线对比。

## 多会话（上游功能，drone_mapping 还没接）

先用同一个 `save_path` 存好几次建图，再在 `General/previous_map` 里列出要加载的旧会话（`名字: 回环阈值`，名字就是当时的 `bagname`），
把新一次的 `bagname` 设成没用过的名字，回放新包。它会在新旧会话之间找回环，最后 `/finish` 时一起做全局 BA。
上游示例见 `config/hesai.yaml`（HILTI 2023）。适合大场地分几次建，但我们还没验证过。

## 注意事项

- **结束靠 `/finish`**：主循环每帧读一次 `/finish`，为 true 就去做全局 BA、存位姿。回放还没处理完就置 true，后面的帧就丢了。
- **设备几乎不动，建不了图**：全局 BA 的平面体素比线程数还少时，上游直接 `exit(0)`（打印 `Too Less Voxel`），不存位姿。
  10-20 实机冒烟（静止 30 s）就是这样。另外位姿少于 100 帧（约 10 s）也不写文件。
- **回放倍速**：它只按消息顺序处理、不看墙钟，只要不丢消息，倍速就不改变结果（0.5 / 1 / 2 倍速逐位相同）。
  处理跟不上时，雷达队列（1000 帧，约 100 s）满了就会丢帧，长包用 1 倍速或更慢。
- **初始化**：开头保持静止几秒；上游说运动中也能初始化，但我们的包都是静止开始的。
- 房间里板端 1 倍速约占 1 个核、内存约 114 MB，全局 BA 4 s；室外大场景的耗时和内存还不知道。

## 上游还有什么

- 上游 README 里的数据集示例（Avia、HILTI 2023 多会话、MARS、MID-360 jungle）没有搬过来，原文见上游仓库。
- `VoxelSLAMPointCloud2` 没收：它是一个 RViz 插件，收到空点云时自动清屏。
