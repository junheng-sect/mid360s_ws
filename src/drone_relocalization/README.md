# drone_relocalization

一次性自动重定位：室内小图（默认 `config/indoor.yaml`），以及室外大图在固定起飞区内启动（`config/outdoor.yaml`，11-05 加，见「室外大图」）。
复用已运行的 MID-360S、FAST-LIO 和重力桥，在未知起始位置/朝向下匹配先验地图。
两个独立静止窗口确认后固定 `drone_map ← drone_odom`，后续只传播里程计，**不做持续地图校正、不向 PX4/MAVROS 发送位姿或控制指令**。

当前默认地图是 `~/test_data/mapping/20261021_110311_indoor`（10-21 房间布局大改后重建，已去人）。
`ACCEPTANCE.md` 和 `test/` 下的回归脚本仍按 10.18 旧图 `20261018_194304_voxelslam` 及旧布局的包记录，旧布局的包不能用来评判新图。
离线软件验收与实机验收分开记录：10-20 离线验收见 `ACCEPTANCE.md`；10-21 在新图上实机静止启动试过 3 次，都能锁定，位置合理，没有用物理真值验收。

## 启动

板端构建：

```bash
source /opt/ros/noetic/setup.bash
cd ~/mid360s_ws && catkin build drone_relocalization
source ~/mid360s_ws/devel/setup.bash
```

未来手持实机测试时，先在一个终端启动现有传感器链路（本命令不启动 MAVROS）：

```bash
~/mid360s_ws/scripts/start_mid360s_fastlio.sh
```

另一个板端终端执行，设备保持静止直至 `/drone_reloc/status` 为 `LOCALIZED`、`valid=true`：

```bash
source ~/mid360s_ws/scripts/ros_net.sh && source /opt/ros/noetic/setup.bash && source ~/mid360s_ws/devel/setup.bash
roslaunch drone_relocalization relocalize.launch map_dir:=$HOME/test_data/mapping/20261021_110311_indoor
```

两个终端使用同一套 ROS 网络环境；已有自定义值时沿用原值，程序不会覆盖 `ROS_MASTER_URI`、`ROS_IP`、`ROS_HOSTNAME`。
不设 `ROS_IP` 时节点按主机名通告，本机 RViz 能列出话题但收不到数据。

双确认成功时终端打印一行 `定位成功，当前位置（地图系，原点=建图起点的雷达 IMU）：x=… y=… z=… m，航向 …°，离地图地面 … m`，
和 `/drone_reloc/pose` 同一条变换链（取最新一帧里程计）：位置是雷达 IMU，航向是机体（drone_base）航向。
雷达相对机体约有 1.6° 安装偏航，所以按建图时的姿势放回起点，航向约 −1.4°（10-21 回放），不是 0°。之后的位置看 `/drone_reloc/pose`。
launch 起重定位节点，默认带上 `car_bridge`（`car:=false` 关闭），把车位姿和统一 map 接进板端真实 TF，见 [car_bridge](../car_bridge/README.md)。
板端不跑任何显示节点，也不再带 RViz 配置；RViz 只在本机看（见下节）。

## RViz 对照显示（本机）

本机（mid360s_ws 仓库根目录）：

```bash
./scripts/run_rviz.sh --reloc
```

显示全在本机 docker 里：RViz 加显示辅助节点 `reloc_display`（`docker/ros-noetic-rviz/local_view`），二维车图、标签和状态文字都在本机生成；
板端只提供真实 TF 和 `/drone_reloc/*`、`/car/status`。Fixed Frame 是 `map`，三组 0.3 m 坐标轴（红 x、绿 y、蓝 z）都是真实帧：
- `map`（青色 `map (0,0,0)` 标签）：统一 map，原点是车 App 的「充电桩」点，x 是车插在桩上时的车头方向（从桩指向房间里），y 朝左，地面 z=0。
- `car_base`（橙色标签）：车体，`/tracked_pose` 的位置和航向，离地 0.2 m（车激光高度）。车不在线或换了地图时，红色提示挂在 map 上方。
  车的位姿由车端 Cartographer 纯定位给出。
- `drone_base`（紫色标签，上方有状态文字）：无人机机体，在 map 里的位姿是一次重定位得到的固定 `drone_map←drone_odom`
  乘上 FAST-LIO 里程计，锁定后不再对地图修正，FAST-LIO 漂多少就差多少。

彩色是先验地图 `/drone_reloc/map`（`drone_map` 系，按高度着色），白色是 FAST-LIO 实时点云 `/cloud_registered`。
定位成功前 `drone_map→drone_odom` 还不存在，三维里只有 map、车和先验地图，无人机和实时点云等定位成功后才出现，状态文字挂在 map 上方。

窗口布局：左栏是 Displays、Views 和摄像头预览 `OV13855 Preview`（同 `fastlio_mid360s.rviz`，相机节点没跑时显示 No Image）；
右侧上面三维，下面是二维车图 `Car map`：按车 App 的画法（车图 x 朝右、y 朝上，map 的 x 正好朝上），画 map 坐标轴、车身框（650×550 mm）和车头、
定位有效时的无人机位置与离地高度，顶部一行是车定位分数和无人机状态。车的栅格图由本机直接从车端 rosbridge 取（只订阅 `/map`）。

同等 rosrun 入口：

```bash
rosrun drone_relocalization relocalize --map ~/test_data/mapping/20261021_110311_indoor
```

旧的 `python3 ~/mid360s_ws/scripts/drone_reloc.py ...` 转发到新入口。没 source 时入口会补 ROS/工作空间环境，并原样保留参数及 ROS 网络设置。

## 一次性定位流程

1. 地图 `map_world.pcd`、`meta.json` 校验并建立索引；不修改点云、元数据或旧缓存。
2. 等待 `/Odometry`、`/cloud_registered` 及 `drone_odom→camera_init`、`body→drone_base` 静态 TF（10-21 改名前的 `world`、`base_link` 也认），检查数据新鲜性和连续静止。
3. 攒2秒点云，二维全局似然场搜索未知 x/y/yaw。同一朝向保留多个空间峰，最多精配8个不同候选。
4. 以候选为初值做6DoF点到面 ICP，先按实际内点支持度选解，再检查内点率、残差、退化、竞争解和相对竖直方向倾角。
   最高支持解退化时拒绝该窗，不能退而接受较弱位置；退化竞争候选仍参与歧义判断。
   歧义同时比较硬内点支持度与残差加权支持度（沿用2cm残差尺度），避免3cm内点截断把拟合更好的竞争解隐藏。
5. 再取一个不重叠的2秒窗口，独立复核第一轮的全部竞争候选，重新计算质量与歧义；不重复耗时的全局粗搜。
   两次结果在5cm/1°内才确认。第一轮不可信或期间运动时清除候选，下一次重新全局搜索。
6. 确认后固定变换值，用后续新鲜里程计发布地图位姿和动态 TF。不会随飞行不断重新匹配地图。

默认参数在 `config/indoor.yaml`：最多3次搜索、等待总限60秒；全采集/确认期间相对锚点位置偏移不得达到3cm、姿态变化不得达到2°；
输入接收超过0.5秒未更新则无效；看门狗使用墙钟，回放 `/clock` 停止也会失效。在线60秒预算包含地图加载耗时；首次重建缓存会占用预算。离线时序验收另列地图加载耗时。
持续运动或初始化里程计自身漂移超门限，也会拒绝，需静止等待稳定后重启；不能为了让旧包通过而放宽门限。

精配允许地图和本次 drone_odom 有至多3°的相对倾角，吸收两次重力对齐之间的小差异；这不是物理水平标定，不能消除沿途非刚体漂移。
地图地面值仅用于搜索高度和墙层，不作为精确离地传感器，也不强行压平室内原图。

## 室外大图（起飞区，11-05 加）

用法前提：每次都在**建图原点附近的起飞圆柱**里启动（水平 3 m、上下 0.5 m 以内；建图时飞机放在标记点上、机头朝标记方向，原点就是那里）。
室内配置直接加载整张室外图会被内存预算拒绝（10-20 园区图 555 万点）。`config/outdoor.yaml` 在室内流程上改了这几处，其余（双窗口确认、静止检测、门限）不变：

| 配置 | 值 | 作用 |
|---|---|---|
| `crop_radius` / `crop_below` / `crop_above` | 20 / 0.5 / 4.0 m | 只用原点水平 20 m、地面 −0.5～+4 m 的地图；地面在这一块里重新拟合（局部地面） |
| `ground_voxel` / `map_voxel` | 0.2 / 0.1 m | 裁出来的地面点、其余点分别降采样，控制查找表内存 |
| `max_range` | 16 m | 实时点云截到水平 16 m（= 20 − 3 − 1），高度同样截到地面 −0.5～+4 m；地图外的点没有对应，会拉低内点占比 |
| `zone_radius` / `zone_height` | 3 / 0.5 m | 全局搜索的候选只在原点 3.5 m 内；解离原点水平超过 3 m 或高度超过 ±0.5 m 就拒绝 |
| `split_ground` | 1 | 地面点占多数时的判据，见下 |
| `max_grid_cells` / `max_expand_bytes` | 1 亿格 / 2 GiB | 裁图后的预算 |

**`split_ground`**：室外有对应的点里约四分之三是地面。地面只约束 z，对水平错位完全不敏感，室内判据在这里会失真：
- 三维平移退化比（最小 / 最大特征值）被地面的 z 约束压到 0.05～0.07，水平两个方向其实都约束得很好（x/y 比值 0.37～0.66）。
  改成只比水平两个方向（`degen_xy`），门限仍是 0.12；z 由地面保证，六自由度秩检查照旧。
- 候选排序和竞争解歧义改成只数对应到竖直面（法向 |n_z| < 0.7，墙、柱）的内点。错开几米的错误解靠地面照样有九成内点，
  合成场景里真解 0.98、错开 2～3.5 m 的两个错解 0.91，歧义 0.93 把真解拒了；只数竖直面后真解 0.19、错解 0.12，歧义 0.64。
- 内点率门限改看竖直面内点率（`fit_v_ok` 0.65，代替 `fit_ok` 0.8）。11-05 实机 10 次测试：
  - 真解的全部点内点率随朝向下降：和建图同朝向约 0.86，±90° 约 0.80，180° 约 0.77。地面换了入射角，掠射噪声变大；地图起点附近的地面主要来自建图时朝向 0° 的首尾静止段。
  - 5 次因 < 0.8 被拒，位置其实都对。错开的竞争解也有 0.65～0.75，降门限分不开对错。
  - 只看竖直面：真解 0.75～0.86，竞争解 0.34～0.55，建图包里起飞区外 22 个错位解 0.33～0.57。
- 二维得分改在 ICP 精配后的位置上算，门限仍是 0.45。原来取全局粗搜格点的分数，墙层多是树丛，差几厘米、一两度分数就掉；同一静止位置相邻窗口在 0.32～0.71 之间跳，精配后稳定在 0.62～0.74。竞争解为 0.28～0.55。
- 内点占比、残差、倾角、歧义等其余门限照旧，数值没有放宽；`split_ground=0`（室内）时行为和以前逐位一致。

**11-05 实机（同一张图，飞机按 30×60 cm 地砖摆放）**：
- 改前 10 次里朝向约 0°、−17°、35° 的 5 次成功；180°（2 次）、+89°（2 次）、−89°（1 次）失败，全是内点率门限（有时加二维得分）。
- 用新判据离线复跑这 10 个包，20 个窗口全部可信；三个失败包走完整启动流程都在数据时间约 20 s 内锁定。
- 位置和摆放一致：(0, −0.30)、(−0.60, 0.30) 等误差 1～2 cm。
- 建图包里离原点 4～28 m 的 20 个窗口全部拒绝，没有误判。
- 包在 `~/test_data/reloc_test/2026110513*`。

第一次载入要建查找表：11-05 室外图裁后 23.1 万点，板端 70 s、峰值内存 2.1 GiB，缓存 `reloc_cache_v2.npz` 240 MB；之后载入 1.7 s。
**出门前先在板端离线跑一次把缓存建好**（在线 60 s 预算包含载入时间）：

```bash
source /opt/ros/noetic/setup.bash && source ~/mid360s_ws/devel/setup.bash
M=~/test_data/mapping/20261105_114605_outdoor
rosrun drone_relocalization relocalize --map $M --config $(rospack find drone_relocalization)/config/outdoor.yaml --bag $M/raw.bag --at 1 --save ""
```

实机测试用一键脚本（开机后一条命令，默认就是这张室外图和 `outdoor.yaml`）：

```bash
~/mid360s_ws/scripts/reloc_test.sh
```

脚本依次做这几件事：
1. 自己开 tmux 会话 `reloc`，ssh 断了也接着跑。
2. 先载入地图：缓存不对就在这里重建，不占用在线 60 s 的预算。
3. 起雷达、FAST-LIO 和重力桥，不送飞控。
4. 默认录包，重定位节点和录包都只读，不改参数。
5. 屏幕上只打印每次搜索的结论、原因，定位成功后每 2 s 打印一次当前位置。
6. Ctrl+C 统一收尾，恢复对时。

日志、结果 json、包和参数在 `~/test_data/reloc_test/<时间>/`。
`--indoor` 换室内图，`--no-record` 不录包，`-h` 看全部选项。

只起重定位节点（传感器链路另外起好、室外没有车所以关掉 car_bridge）：

```bash
roslaunch drone_relocalization relocalize.launch map_dir:=$HOME/test_data/mapping/20261105_114605_outdoor \
  config:=$(rospack find drone_relocalization)/config/outdoor.yaml car:=false
```

## 输入、输出与失效

| 接口 | 类型 / 语义 |
|---|---|
| `/Odometry` | `nav_msgs/Odometry`，`camera_init → body`，位置原点为雷达 IMU |
| `/cloud_registered` | `sensor_msgs/PointCloud2`，`camera_init`，FAST-LIO 已去畸变注册的点云 |
| `/tf_static` | 必须包含 `drone_odom→camera_init` 和 `body→drone_base`（旧名 `world`、`base_link` 也认，第一次遇到时打一条警告），运行中不允许改变 |
| `/drone_reloc/map` | latched `PointCloud2`，先验点云，`drone_map` 系 |
| `/drone_reloc/pose` | `PoseStamped`，`drone_map` 中的 `drone_base` 姿态，沿用记录的安装 TF |
| `/drone_reloc/result` | latched `String` JSON v2；只有双确认后 `ok=true, confirmed=true`；首轮可有 `candidate_ok=true` 但 `ok=false` |
| `/drone_reloc/status` | latched + 1Hz `String` JSON：时间戳、state、valid、why、次数、确认数、`tracking=false` |
| `/tf` | 动态 `drone_map→drone_odom`，变换值固定，只随新鲜里程计推进时间戳 |

本套安装 TF 的平移为0，因此 drone_base 原点仍是雷达 IMU，不代表已补偿到机体重心。

JSON v2 中 `transform.translation` 为 xyz，`transform.q_xyzw` 为四元数，均表示 `drone_map ← drone_odom`：
`p_map = R_map_world @ p_world + t_map_world`。
兼容统计字段 `T=[x,y,z,yaw]` 仅为投影，带 `T_is_yaw_projection=true`，存在roll/pitch时不能用T重建真实变换。
`height` 是相对地图记录地面的 z 差，不是测距真值。在线未确认候选的 `ok` 与离线单窗 `ok` 语义不同：
离线表示该窗口几何判据通过，在线表示双窗口最终确认；使用在线结果还必须看当前 status。

断流、时间回退/重复、初始化 TF 改变后置 `STALE`/`FAILED`，清除有效解，停止pose和TF，更新 latched result 为无效。
不会在输入恢复后复用旧解，需重启重定位节点。TF监听器可能缓存旧TF，不能仅以 `lookup_transform(..., Time(0))` 成功判断有效；
消费者必须同时检查 status.valid 以及 pose/status 的接收新鲜性。进程崩溃时也应按接收超时拒绝缓存消息。

## 离线复现

不需要 ROS master，不启动雷达、FAST-LIO 或飞控。原始包需包含雷达、Odometry和上述两段静态TF。
离线读取实际外参，对逐点时间插值去畸变；不会把多个扫描直接堆在一个时刻。

```bash
source /opt/ros/noetic/setup.bash && source ~/mid360s_ws/devel/setup.bash
rosrun drone_relocalization relocalize --map ~/test_data/mapping/20261021_110311_indoor --bag ~/test_data/mapping/20261021_110311_indoor/raw.bag --at 1
rosrun drone_relocalization relocalize --map ~/test_data/mapping/20261021_110311_indoor --bag ~/test_data/mapping/20261021_110311_indoor/raw.bag --eval --step 3 --out /tmp/reloc_eval.json
rosrun drone_relocalization relocalize --map ~/test_data/mapping/20261021_110311_indoor --bag ~/test_data/mapping/20261021_110311_indoor/raw.bag --startup-eval --at 1 --out /tmp/reloc_startup.json
```

示例用的是建图自身的原始包（目前新布局下只有这一段开头静止够久的包），结果偏乐观；`flight_raw_000001.bag` 是旧布局的包。

`--eval` 是各窗口独立的几何回归，移动段也可评估；不是完整启动验收。
`--startup-eval` 使用同一个启动状态机，并在每次实际计算耗时内继续推进历史输入，真实检查期间是否运动。
旧包可能在确认完成前已起飞/被拿起，这种情况下预期拒绝，不等于单窗无法定位。

其他参数：`--window`、`--tries`、`--still-timeout`、`--config`、`--yaw-perturb`、`--save`（在线结果目录，空串不存）。
退出码0=通过/正常退出，1=未确认定位，2=用法错误，3=地图/数据错误，130=中断。

地图使用新文件 `reloc_cache_v2.npz`：指纹包含 PCD/元数据内容、完整算法配置与索引版本；损坏或不符即重建，原子替换。
只读地图可不存缓存继续工作；超内存预算会在分配前报错。室内配置不裁图，只适合小室内图；大图用 `config/outdoor.yaml` 按起飞区裁图。
同一张图换配置会重建缓存（缓存文件只有一份）。

## 软件验收

```bash
python3 -B -m unittest discover -s ~/mid360s_ws/src/drone_relocalization/test -v
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B ~/mid360s_ws/src/drone_relocalization/test/regression.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B ~/mid360s_ws/src/drone_relocalization/test/acceptance.py --out ~/test_data/reloc_acceptance/20261020/acceptance.json
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B ~/mid360s_ws/src/drone_relocalization/test/ros_replay.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B ~/mid360s_ws/src/drone_relocalization/test/bench_map.py
```

最后一项自行创建 `127.0.0.1:11531` 独立master，已占用就退出；只发布历史点云构成的模拟静止流及测试TF/里程计，
检查定位、地图消息、机体姿态、动态TF及断流/时间回退/TF变化及回放时钟冻结保护，结束只清理自己启动的进程。
报告在 `~/test_data/reloc_acceptance/20261020/`，验收结果以 `ACCEPTANCE.md` 为准。所有统计是软件回归/自洽性证据，不是实机或飞行精度验收。
