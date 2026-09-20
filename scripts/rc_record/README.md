# RC 开关触发录包（rc_record）

> 位置：`/home/orangepi/mid360s_ws/scripts/rc_record/`（属于 mid360s_ws 仓库）。
> 融合快速启动：`cd /home/orangepi/mid360s_ws/scripts && ./fusion.sh`
> —— 一条 `launch/fusion.launch` 起 MAVROS + 雷达/FAST-LIO + 本录包节点，
> 用 `RC_BAG_PREFIX`/`RC_BAG_DIR` 把包名前缀（默认 `noprop`）与输出目录传进来。
> 逐项体检入口：`./start_handheld_test.sh`（会 ping 雷达、核对飞控参数、测频）。
> 两个入口共用同一把锁，互相排斥。下面的说明也可单独手动启动。

`bag_recorder_node.py`：把遥控器某个通道（默认**通道 6**）当录包开关，拨到触发位就开一个
标准 ROS1 bag，离开触发位后再保留 1 s 尾巴关包。配合飞控 `SDLOG_MODE=3`（AUX1）可以做到
**一个开关同时开始/停止 bag 与 ulog**。

实现风格参照板端 `orange` 工作空间 `vz_step_control` 的 `offboard_bag_recorder`：
直接用 rosbag Python API 写包（不 fork `rosbag record`），策略抽成不依赖 ROS 的纯模块，
bag 名用六位序号 + 状态文件，崩溃最多留空号、绝不复用编号。

## 文件

| 文件 | 说明 |
|---|---|
| `bag_recorder_node.py` | 节点本体：订阅 `/mavros/rc/in` 判定触发，写 bag，发布状态 |
| `rc_record_policy.py` | 纯策略状态机（去抖 / 尾巴 / 防半截包），无 ROS 依赖，可离线测 |
| `sequence_allocator.py` | 六位序号 bag 路径分配（锁 + 原子写 + fsync） |
| `rc_record.yaml` | 默认配置（话题清单、阈值、目录、前缀） |
| `start_rc_record.sh` | 启动包装：加载 yaml + `exec python3 bag_recorder_node.py`；也是 launch 里的 `<node type>` |
| `package.xml` | 仅用于让 rospack/roslaunch 定位本目录（不参与 catkin 构建，见文末备注） |
| `test_rc_record_policy.py` | 策略与序号分配器的离线测试（27 项断言） |
| `test_rc_record_e2e.sh` | 端到端测试：隔离 Master + 假 RC/位姿发布器，自动翻转开关验收发 |

## 与飞控 ulog 配对（关键）

飞控侧这条链路已经就绪：`RC_MAP_AUX1 = 6`（AUX1 已经映射到通道 6）。
PX4 logger 在 `SDLOG_MODE=3` 下的判据是源码里的
`desired_state = (manual_control_setpoint.aux1 > 0.3f)`，而 `aux1` 由 PWM 归一化
（1000→−1、1500→0、2000→+1），**约等于 PWM > 1650 µs 才是「记录位」**。

所以：

- 本节点默认 `active_high: true`、`rc_threshold: 1650`，与 ulog 同向 —— 开关拨到
  **PWM 大的那一侧**，bag 与 ulog 一起开始；离开则一起停。
- `SDLOG_MODE` 是 **reboot_required** 参数，改完要重启飞控才生效：

```bash
rosservice call /mavros/param/set "{param_id: 'SDLOG_MODE', value: {integer: 3, real: 0.0}}"
# 或 QGC 里把 SD Logging 的 Logging Mode 设成 "while manual input AUX1 >30%"
```

- 若你的遥控器上该开关「拨下」输出的是低电平，ulog 不会开始。两个选择：
  1. 用拨上去的那一侧当录制位（推荐，改遥控器习惯而已）；
  2. 在遥控器里把该通道**反向**（这样拨下 = 高电平），本节点不需改动；
  3. 只想让 bag 单独触发、不在乎 ulog，则设 `active_high: false`
     （但此时 bag 与 ulog 不同步，本节点的启动日志会显示 `active_high=low`）。

**先确认极性**（起飞前必做，一次就够）：

```bash
rostopic echo /mavros/rc/in    # 看 channels[5]（通道 6）在两个位置各是多少 us
```

节点自己也会在 PWM 变化 ≥25 µs 时限频打印 `通道 6 PWM=xxxx us -> 触发=True/False`，
直接看它的日志就能对照出来。

## 用法

```bash
cd /home/orangepi/mid360s_ws/scripts/rc_record

# 1) 先有 ROS Master：由一键脚本拉起 MAVROS（或手动 roslaunch mavros px4.launch）
# 2) 前台启动录包节点
./start_rc_record.sh

# 临时覆盖输出（不改 yaml）
RC_BAG_PREFIX=handheld RC_BAG_DIR=/home/orangepi/bags ./start_rc_record.sh

# 一体化（推荐）：MAVROS + 雷达/FAST-LIO + 本节点一起起，默认前缀 noprop
cd /home/orangepi/mid360s_ws/scripts && ./fusion.sh
# 逐项体检版（ping 雷达 / 核对飞控参数 / 测频 / 等 EKF）：
cd /home/orangepi/mid360s_ws/scripts && ./start_handheld_test.sh
```

状态话题（便于在另一个终端观察）：

```bash
rostopic echo /rc_record/status      # state=REC/IDLE bags=N last=<当前包> rc_valid=... healthy=...
rostopic echo /rc_record/bag_path    # latched："OPEN <path>" / "CLOSED <path>"
```

产出：`/home/orangepi/bags/handheld_000001.bag`、`000002`、… 序号状态文件
`/home/orangepi/bags/.handheld_sequence`（与今天的 `20260919_*.bag` 等时间戳命名的包不冲突）。

## 参数（`rc_record.yaml`，命令行也可用 `_name:=value` 覆盖）

| 参数 | 默认 | 说明 |
|---|---|---|
| `rc_topic` | `/mavros/rc/in` | 触发源话题 |
| `rc_channel` | `6` | 1-based，与飞控 `RC_MAP_AUX1` 一致 |
| `rc_threshold` | `1650` | µs 阈值 |
| `active_high` | `true` | true: PWM ≥ 阈值触发；false: PWM ≤ 阈值触发 |
| `confirm_samples` | `3` | 连续 N 拍一致才认账（去抖，20 Hz 下约 0.15 s） |
| `release_tail_s` | `1.0` | 松手后继续录的尾巴；期间重新拨下则继续同一个包 |
| `rc_warn_timeout_s` | `5.0` | 多久收不到 RC 就告警 |
| `stop_on_rc_loss` | `false` | false: 丢遥控也不截断正在录的包 |
| `bag_directory` / `bag_prefix` | `/home/orangepi/bags` / `handheld` | 输出位置与包名前缀 |
| `record_raw_cloud` | `false` | true 额外录 `/livox/lidar`（约 +5 MB/s，需先 source mid360s_ws/devel） |
| `status_rate` | `2.0` | `/rc_record/status` 频率 |
| `topics` | 13 个（见 yaml） | 录制话题；类型必须在节点的登记表里，否则启动时跳过并报错 |

## 行为约定

- 只有先看到过「有效且未触发」的电平才允许开包 —— 节点重启/respawn 时若开关已在触发位，
  不会凭空生成半截包，要等你先松开再拨下。
- 去抖 + 尾巴：抖动不会产生碎片包；一次连续的拨下→松手只得到一个包。
- RC 丢失不截断正在录的包（`stop_on_rc_loss: false` 时），链路恢复后照常按松手关包。
- 尾巴到期由 5 Hz 心跳驱动，不依赖 RC 数据，所以丢遥控也不会漏关包。
- 写包失败或开包失败 → 记录为 fatal，停止接受新触发（并 shutdown 节点），不会静默丢数据。
- 收到 SIGINT（Ctrl+C / start_handheld.sh 收尾）会正常 close 包写索引。
- bag 内消息时间戳优先用 `header.stamp`（与 `rosbag record` 同口径），无 header 的用接收时刻。

## 验证

```bash
python3 test_rc_record_policy.py     # 纯逻辑，27 项断言
./test_rc_record_e2e.sh              # 隔离 Master + 假 RC，验证真的开/关包
```

E2E 脚本会在临时目录里用 `channels[5] = 1000/2000/1000/2000/1000` 的脚本翻转开关，
期望得到 **2 个** bag，各含 pose 消息且都已正常收尾（无残留 `.bag.active`）。
换端口跑：`./test_rc_record_e2e.sh 11378`（避免和别的 Master 撞）。

## 备注

- 这仍然不是 catkin 包：不在 `src/` 下，不参与 `catkin build`，也不产出任何可执行文件。
  为了能进 `launch/fusion.launch` 的 `<node>`（ROS1 要求 `pkg` 能被 rospack 找到、`type`
  是包目录里的可执行文件），目录里放了一个**仅用于定位**的 `package.xml`，`type` 直接
  复用 `start_rc_record.sh`。代价是启动前要把 `scripts/` 加进 `ROS_PACKAGE_PATH`：
  `fusion.sh` 里已经做了，手动 `roslaunch` 时要自己 `export`，否则报
  `cannot find package rc_record`。
- 触发与 ulog 都依赖飞控能收到 RC（`COM_RC_IN_MODE=3` 已开）。若遥控器没绑，
  `/mavros/rc/in` 不会有数据，节点会告警「收不到 RC 数据」且不录任何包。
