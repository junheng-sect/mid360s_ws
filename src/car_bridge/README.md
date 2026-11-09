# car_bridge

把 M65 小车的位姿接进板端 TF 树。只读车端 rosbridge（`192.168.31.231:9090`），**不给车发任何指令**。
板端只做这一件事；二维车图、标签这类显示都在本地 docker RViz（`docker/ros-noetic-rviz`，`run_rviz.sh --reloc`）。

```bash
roslaunch car_bridge car_bridge.launch          # 单独起
roslaunch drone_relocalization relocalize.launch  # 重定位默认带上它（car:=false 关）
```

## TF 和话题

```
map ──静态──> drone_map ──重定位──> drone_odom ──> camera_init ──> body ──> drone_base
 └───静态──> car_map ──/tracked_pose──> car_base
```

| 输出 | 说明 |
|---|---|
| `/tf_static` `map→drone_map`、`map→car_map` | 统一 map 标定，来自 `config/map_frame.yaml`；`--drone-map` 不是标定用的那张图时不发 `map→drone_map` |
| `/tf` `car_map→car_base` | 车 `/tracked_pose`（Cartographer 纯定位，`base_footprint`）的 x、y、yaw，z 固定 `car_base_height`=0.20 m（车激光高度），10 Hz |
| `/car/pose` | `PoseStamped`，`map` 系下的 `car_base`，和 TF 同一时间戳 |
| `/car/status` | latched JSON，1 Hz：`online`、`age_s`（距上一帧车位姿的秒数）、`map_id`/`map_name`/`map_ok`、`score`（车 `/local_global_score` 最后一项）、`valid`、`why` |

- 车和板端时钟不同步，时间戳用板端收到的时间；rosbridge 节流 0.1 s，所以位姿最多晚 0.1 s 左右。
- 车的 `/current_floormap` mapId 不是标定时的 `1792549441088`（车换了图）时，不发 `car_base` 和 `/car/pose`，`/car/status` 写原因。
- 车断线后 TF 停在最后一帧，用位姿前先看 `/car/status` 的 `valid` 和 `age_s`。
- 无人机和车的相对位姿直接查 TF：`drone_base` ↔ `car_base`（无人机需已重定位，`drone_map→drone_odom` 存在）。

## 统一 map 和标定

- 原点是车 App 的「充电桩」点，z=0 在地面；x 是车停在桩上时的车头方向（从桩指向房间里），y 朝左。
- `config/map_frame.yaml`：map←drone_map、map←car_map，来自 10-21 车图「10.21」和无人机图 `20261021_110311_indoor`
  的离线配准（car_map←drone_map = (0.6803, 0.8924, −1.8151°)，约 2 cm / 0.3°）。换了任一张图都要重新配准。
- `car_base_height`：car_base 离地高度，取车激光高度 0.20 m（车 TF 里 `base_footprint→scan`）。用相对位姿时注意它比车的
  `base_footprint` 高 0.20 m。

### 室外（`config/map_frame_outdoor.yaml`，无人机图 `20261105_114605_outdoor`）

- map 就是建图时无人机机头为 x 的 FLU 系：x、y、yaw 和 drone_map 相同，只把 z=0 放到起点下方的地面（起点 1 m 内地面拟合
  z=−0.151，所以 map←drone_map = (0, 0, 0.151, 0°)）。
- 室外还没有车图，`car_map_id`、`map_from_car_map` 写 `null`：不发 `map→car_map`、`car_base` 和 `/car/pose`，
  `/car/status` 的 why 是 `no car map registered for this site`。车建好室外图后按室内的做法配准再填上（两项要同时填）。
- 重定位时带上：

```bash
roslaunch drone_relocalization relocalize.launch map_dir:=$HOME/test_data/mapping/20261105_114605_outdoor \
  config:=$(rospack find drone_relocalization)/config/outdoor.yaml map_frame:=$(rospack find car_bridge)/config/map_frame_outdoor.yaml
```

## 测试

```bash
python3 -B -m unittest discover -s ~/mid360s_ws/src/car_bridge/test -v
```
