# drone_relocalization

室内一次性自动重定位。复用已运行的 MID-360S、FAST-LIO 和重力桥，在未知起始位置/朝向下匹配先验地图。
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
export ROS_IP=192.168.31.32 && source /opt/ros/noetic/setup.bash && source ~/mid360s_ws/devel/setup.bash
roslaunch drone_relocalization relocalize.launch map_dir:=$HOME/test_data/mapping/20261021_110311_indoor
```

两个终端使用同一套 ROS 网络环境；已有自定义值时沿用原值，程序不会覆盖 `ROS_MASTER_URI`、`ROS_IP`、`ROS_HOSTNAME`。
不设 `ROS_IP` 时节点按主机名通告，本机 RViz 能列出话题但收不到数据。

双确认成功时终端打印一行 `定位成功，当前位置（地图系，原点=建图起点的雷达 IMU）：x=… y=… z=… m，航向 …°，离地图地面 … m`，
和 `/drone_reloc/pose` 同一条变换链（取最新一帧里程计）：位置是雷达 IMU，航向是机体（drone_base）航向。
雷达相对机体约有 1.6° 安装偏航，所以按建图时的姿势放回起点，航向约 −1.4°（10-21 回放），不是 0°。之后的位置看 `/drone_reloc/pose`。
launch 起重定位节点和显示节点 `reloc_view`（`view:=false` 关闭）、`car_view`（`car:=false` 关闭，见下节），可用 `rviz:=true` 在板端起 RViz；板端是 llvmpipe 软件渲染，会和重定位抢 CPU，建议在本机看（见下节）。

## RViz 对照显示

本机（mid360s_ws 仓库根目录）：

```bash
./scripts/run_rviz.sh --reloc
```

彩色是先验地图（按高度着色），白色是 FAST-LIO 实时点云 `/cloud_registered`；三组 0.3 m 坐标轴（红 x、绿 y、蓝 z）：
- 带青色 `map (0,0,0)` 标签的是统一 map：原点是车 App 的「充电桩」点，x 是车插在桩上时的车头方向（从桩指向房间里），y 朝左，地面 z=0。
- 带橙色 `car_base` 标签的是车体（`/tracked_pose` 的位置和航向），固定画在离地 0.2 m（车激光高度）。车 2 s 没数据或换了地图时标签变红并写明原因。
  车的位姿由车端 Cartographer 纯定位持续拿激光对地图修正。
- 带紫色 `drone_base` 标签、上方有状态文字的是无人机机体。它在 map 里的位姿是一次重定位得到的固定 `drone_map←drone_odom`
  乘上 FAST-LIO 里程计（`drone_odom` 就是无人机的 odom 系），锁定后不再对地图修正，FAST-LIO 漂多少就差多少。

窗口布局：左栏是 Displays、Views 和摄像头预览 `OV13855 Preview`（同 `fastlio_mid360s.rviz`，相机节点没跑时显示 No Image）；
右侧上面三维，下面是二维车图 `Car map`：按车 App 的画法（车图 x 朝右、y 朝上，map 的 x 正好朝上），画 map 坐标轴、车身框（650×550 mm）和车头、
定位有效时的无人机位置与离地高度，顶部一行是车定位分数和无人机状态。
定位前按「起点就是地图原点」画实时点云，无人机不在地图原点起步时两者错位；`/drone_reloc/status` 有效后两者重合；
定位失效（STALE/FAILED，或 status 2.5 秒没更新）又退回错位。

实现只服务显示，不改真实输出：`reloc_view` 把 `/drone_reloc/map` 转发成 `/drone_reloc/view/map`（`drone_map_view` 系），
在 `/tf_static` 上发 `drone_odom→drone_map_view`，未定位为单位变换，定位有效时为 `drone_map←drone_odom` 的逆。
真实的 `/tf` `drone_map→drone_odom` 和 `/drone_reloc/pose` 仍只在定位有效时由重定位节点发布；不要用 `drone_map_view` 做导航或控制。
状态文字话题 `/drone_reloc/view/status_text`（英文，RViz 字体不支持中文）。

车和 map 由 `car_view` 节点显示，同样只服务显示：
- 只经车端 rosbridge（默认 `car_rosbridge:=192.168.31.231:9090`）订阅 `/tracked_pose`、`/map`、`/current_floormap`、`/local_global_score`，不给车发任何东西；车不在线时每 2 s 重连，不影响重定位。依赖 `websocket-client`（pip）。
- 标定在 `config/map_frame.yaml`（`map_frame:=` 可换）：map←drone_map、map←car_map，来自 10-21 车图 10.21 与无人机图 `20261021_110311_indoor` 的离线配准（约 2 cm / 0.3°）。
- 在 `/tf_static` 发 `drone_map_view→map_view→car_map_view`，在 `/tf` 发 `car_map_view→car_view`；`/car_view/labels` 是文字标签，`/car_view/map_image/compressed` 是 2 Hz 的 PNG 二维车图。
- 车 `/current_floormap` 的 mapId 不是标定时的 `1792549441088` 时不发车的 TF；`map_dir` 不是标定用的无人机图时 map 和车都不发。换了任一张图都要重新配准、改 `map_frame.yaml`。
- 这些帧都带 `_view` 后缀，真实 `/tf` 里还没有 `map` / `car_map`，不要拿来导航或控制。

没有实机数据时可用回放演示（板端，私有 master `192.168.31.32:11531`；实时点云被人为挪开 1.3 m、转 60°，前 25 秒模拟手持晃动）：

```bash
source /opt/ros/noetic/setup.bash && source ~/mid360s_ws/devel/setup.bash
python3 -B ~/mid360s_ws/src/drone_relocalization/test/view_replay.py --demo
```

本机另开终端：`./scripts/run_rviz.sh --reloc --ros-master http://192.168.31.32:11531`。不加 `--demo` 是自动断言（只用 127.0.0.1）。

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
只读地图可不存缓存继续工作；超内存预算会在分配前报错。本实现只交付小室内图，不用于直接加载园区地图。

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
