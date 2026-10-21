# drone_mapping

无人机先验地图建图。实机：拉起 MID-360S 驱动、FAST-LIO2 和重力对齐桥接，录原始包；按 Ctrl+C 结束后，
离线对原始包跑 **Voxel-SLAM**（带局部 BA、回环和全局 BA），再用它的轨迹把原始点拼成 `world` 系地图。
产出放在 `~/test_data/mapping/<时间>[_名字]/`，`map_world.pcd` + `meta.json` 的格式和以前一样，`drone_reloc.py --map` 直接用。

为什么换成 Voxel-SLAM（10-19/20，同样三个原始包离线对比）：

| 指标 | FAST-LIO | Voxel-SLAM |
|---|---|---|
| 闭环差（两次手持回到起点） | 114 / 78 mm | 2 / 2 mm |
| 面厚度中位 | 11.2–11.7 mm | 5.6–7.0 mm |
| 跨次一致性 p90 | 16–23 mm | 约 7 mm |

实时 FAST-LIO 在起点静止的十几秒里就会漂 3–5 cm、yaw 约 1°，用它建的旧图整体偏了约 9 cm。
Voxel-SLAM 不发 `nav_msgs/Odometry`，**飞行定位仍用 FAST-LIO**；实机建图时 FAST-LIO 照样跑（给 `raw.bag` 录 `/Odometry`、
终端里报走了多远），只是不再存它的点云。

## 用法

```bash
source /opt/ros/noetic/setup.bash && source ~/mid360s_ws/devel/setup.bash
rosrun drone_mapping build_map                      # 实机：等提示「初始化完成」再走，回到起点按 Ctrl+C
rosrun drone_mapping build_map --name lab           # 输出目录名加后缀
rosrun drone_mapping build_map --bag RAW.bag        # 用录好的原始包离线重建（map_flight.sh 落地后就是这样调的）
rosrun drone_mapping build_map --post DIR           # 只重新拼图（比如换 --voxel），不重跑 SLAM
rosrun drone_mapping build_map --post DIR --carve 0 # 重新拼图但不去人（默认去掉设备走过空间里的人，见下）
rosrun drone_mapping build_map --algo fastlio ...   # 旧做法：FAST-LIO 存 scans.pcd 再摆正
```

`scripts/build_map.py`（工作空间根目录下）是转发到这里的兼容入口，老命令照样能用。
没 source 也可以直接执行 `src/drone_mapping/scripts/build_map`，它会自己套一层环境。

流程（默认 `--algo voxelslam`）：

1. 前置检查（同以前）：雷达网口和 ping、没有 ROS master 和冲突进程、磁盘、进程锁、等 chrony 对上；
   `--bag` 离线时不要求没有 master（Voxel-SLAM 用自己的 master）。
2. 实机：驱动 + FAST-LIO + 重力桥 + `rosbag record` 录 `raw.bag`；FAST-LIO 初始化完成后再静止 3 s 才提示可以走。
3. Ctrl+C（SIGTERM、ssh 断开的 SIGHUP 也一样）：先停录包，再停 roslaunch，`raw.bag` 落盘。
4. 离线 Voxel-SLAM（`drone_mapping/voxelslam.py`）：在 127.0.0.1 的空闲端口上起独立 roscore，
   `roslaunch drone_mapping voxelslam.launch`，1 倍速 `rosbag play`，回放结束后逐帧输出 10 s 不再增加就置 `/finish`，
   等全局 BA 存出位姿 `alidarState.txt`。存下的位姿少于雷达帧数的 90% 当作丢帧失败。
   这一步再按 Ctrl+C（或 SIGTERM）就放弃建图，`raw.bag` 保留；SIGHUP 不打断。
5. 拼图（`drone_mapping/assemble.py`）：原始点按和预处理一样的规则取点，按 Voxel-SLAM 轨迹逐点插值去畸变，
   按它估的重力摆正，去人（`--carve`，见下），0.05 m 体素降采样；地面拟合、首尾位姿差、预览图。

离线建图耗时 ≈ 原始包时长 + 约 30 s（起节点、处理完再等 10 s、全局 BA、拼图 7–11 s；106 s 的包 136 s，155 s 的包 192 s），
板端约占 1 个核。
**只要不丢消息，回放倍速不改变结果**：Voxel-SLAM 只按消息顺序处理，0.5 / 1 倍速三个包逐位相同。
`--rate` 可以加快回放，但处理跟不上时订阅队列（雷达 1000 帧）会溢出丢帧，长包慎用。

### 去人（`--carve`，10-21 加）

手持建图时，拿设备的人会被扫进图里（10-21 室内图离轨迹 15 cm 内有 1201 个点，静止物体不可能在设备走过的地方）。
拼图时按 0.1 m 格子判断：格子中心离轨迹**水平 0.25 m、上下 0.12 m 以内**（扁椭球），并且在**不到 5 个不同的秒里**被看到，
整格的原始点都去掉，再做 0.05 m 体素降采样。`--carve R` 改水平半径，`--carve 0` 关闭；上下半高、秒数、格子大小是
`assemble.py` 里的 `CARVE_H` / `CARVE_SECS` / `CARVE_CELL`，结果记在 `meta.json` 的 `carve`。

- 为什么要看秒数：只按距离判断会误删。10-21 那次设备从一个约 1 m 高的静止物体旁边擦过，它全程 144 s 都看得到，
  按秒数就留下了。格子用 0.1 m 而不是 0.05 m：静止物体边缘的 0.05 m 小格只偶尔被扫到，会被削掉约 35 个点。
- 为什么上下只取 ±0.12 m：起点时雷达 IMU 离地约 0.16 m，地面不能切；比设备低的东西受 MID-360 视场（−7°～52°）限制，
  本来就只在走近时短暂可见，没法按秒数区分人和物。**要求起点时雷达离地高于 0.12 m**，否则起点附近的地面可能被切掉。
- 10-21 室内图（`20261021_110311_indoor`）：去掉 1341 个体素（1.4%），被去掉的点 z 0.91–1.61 m，离墙最近 0.21 m；
  离轨迹 15 cm 内的点 1201 → 200，擦边的静止物体保留；地面拟合和首尾差不变。上午同一房间的图（10:39）按同样规则去掉 410 个体素。
- 只能去掉人在设备高度、贴着轨迹的部分（手臂、肩、头）。人的身体大多在设备后面 0.3–0.5 m，那部分还留着。
  加大范围试过，效果有限还会误删：

  | 水平 / 上下 | 去掉 | 疑似误删静止点 | 削到地面 | 离轨迹 0.6 m 内短暂可见点剩余（原 6617） |
  |---|---|---|---|---|
  | 0.25 / 0.12 m（默认） | 1341 | 25 | 0 | 5307 |
  | 0.40 / 0.20 m | 2433 | 37 | 66 | 4327 |
  | 0.50 / 0.30 m | 3585 | 129 | 195 | 3394 |
  | 0.60 / 0.40 m | 4653 | 238 | 252 | 2590 |

  疑似误删是拿上午那张图交叉检查的：被删的点 7 cm 内在上午那次有持续可见（≥ 6 s）的点。
  要把人整个去掉，得按激光有没有穿过那块空间来判断（自由空间检测），不能只看离轨迹多远。
- 飞行包（`map_flight.sh` 落地后 `--bag`）没有人，轨迹附近本来就空，默认也开着：10-19 那次飞行包
  （`flight_raw_000001`，86 s）实测轨迹附近一个格子都没有，地图一点不变。

## 产出

| 文件 | 内容 |
|---|---|
| `raw.bag` | 实机：`/livox/lidar` `/livox/imu` `/tf_static` `/fastlio/gravity_init` `/Odometry` |
| `map_world.pcd` | `world` 系（z 朝上，原点 = 起点时的雷达 IMU，yaw 不动），0.05 m 体素，字段 `x y z intensity` |
| `preview.png` | 俯视、侧视和轨迹 |
| `meta.json` | `algo`、点数、地面拟合（起点离地高度、残余倾斜）、首尾位姿差、Voxel-SLAM 帧数和各段耗时、去人（`carve`）、git 版本 |
| `voxelslam_state.txt` | Voxel-SLAM 位姿（每行 `t x y z qx qy qz qw …`，第 18–20 列是重力）；`--post` 靠它重新拼图 |
| `voxelslam.yaml` / `voxelslam.log` | 本次实际加载的参数、节点和回放的输出 |
| `build_map.launch` / `params.yaml` / `roslaunch.log` / `rosbag.log` / `build_map.log` | 实机链路 |

`--algo fastlio` 时另有 `scans.pcd`（FAST-LIO 原样输出，`camera_init` 系，稠密），离线时录 `odom.bag`（只有后三个话题），没有 `voxelslam_*`。
`--keep-frames` 保留 Voxel-SLAM 的逐帧点云 `voxelslam_tmp/`（默认成功后删掉）；失败时一律保留，方便排查。

## 文件

```
config/voxelslam_mid360s.yaml   Voxel-SLAM 参数（上游 mid360.yaml，数值未改；拼图也从这里读外参、blind、point_filter_num）
launch/voxelslam.launch         起 voxelslam 节点，save_path / bagname / is_save_map 在这里覆盖
scripts/build_map               入口
src/drone_mapping/build.py      命令行、实机录包、FAST-LIO 那一路的后处理
src/drone_mapping/voxelslam.py  离线跑 Voxel-SLAM
src/drone_mapping/assemble.py   轨迹 + 原始点 → map_world.pcd / meta.json / preview.png
src/drone_mapping/mapio.py      PCD 读写、体素降采样、地面拟合、预览图（drone_reloc.py 也用）
test/test_mapping.py            单元测试
```

依赖 `src/voxel_slam`（Voxel-SLAM 源码）和 `third_party/gtsam-4.0.3`，由 `scripts/build.sh` 一起编译。

## 测试

```bash
cd ~/mid360s_ws && python3 -B -m unittest discover -s src/drone_mapping/test
```

## 已知限制

- 设备几乎不动的包建不了图：上游在全局 BA 里平面体素太少时直接 `exit(0)`（`Too Less Voxel`），不存位姿。
  10-20 实机冒烟（设备静止 30 s）就是这样；脚本会报这个原因并保留 `raw.bag`。正常走一圈、只在起飞点附近小范围飞的包都没问题。
- 没有真值，精度结论都来自自洽指标（闭环差、面厚度、跨次一致性）；只在一个约 6.3 × 3.5 m 的房间验证过。
- Voxel-SLAM 摆正后，地面稳定地倾约 0.6°（三次建图同一方向），是地面真有坡还是设备误差还没量；地图不按地面二次摆正，
  因为 10-20 试过摆正版，drone_reloc 的可信次数反而变少。
- drone_reloc 的门限是 10-18 用 FAST-LIO 旧图调的；换新图后飞行包的「有对应的内点率」最低 0.82，离门限 0.80 很近。
- 去人只去掉贴着轨迹、和设备同高的那部分人，身体其余部分还在图里（见「去人」一节）。
