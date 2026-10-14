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
| `test_rc_record_e2e.sh` | 端到端测试：隔离 Master + 假 RC/位姿/control_mode 发布器，自动翻转开关验收发，并检查 latch 话题开包补写 |

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
RC_BAG_PREFIX=handheld RC_BAG_DIR=/home/orangepi/test_data/bags ./start_rc_record.sh

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

产出：`/home/orangepi/test_data/bags/handheld_000001.bag`、`000002`、… 序号状态文件
`/home/orangepi/test_data/bags/.handheld_sequence`（与今天的 `20260919_*.bag` 等时间戳命名的包不冲突）。

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
| `bag_directory` / `bag_prefix` | `/home/orangepi/test_data/bags` / `handheld` | 输出位置与包名前缀 |
| `record_raw_cloud` | `false` | true 额外录 `/livox/lidar`（约 +5 MB/s，需先 source mid360s_ws/devel） |
| `status_rate` | `2.0` | `/rc_record/status` 频率 |
| `topics` | 14 个（见 yaml） | 录制话题；类型必须在节点的登记表里，否则启动时跳过并报错 |
| `latched_topics` | `/front_marker_control/control_mode` | 只在变化时发布的 latch 话题：平时缓存最新一条，开包时先补写当前值（时间记为开包时刻）；须同时在 `topics` 里 |

已从默认清单移除 4 个重复话题（类型仍在登记表里，需要时在 yaml 加回即可）：
`/fastlio/vision_pose_cov` 与 `/mavros/vision_pose/pose_cov` 逐条相同（桥接本地回显）；`/tf` 只有 camera_init→body，与 `/Odometry` 相同；`/tf_static` 是 latched 消息，节点启动时就收到、开包后不会再来，实际录不到；`/mavros/estimator_status` 在飞控 ulog 的 estimator_status 里更全。

## 控制话题记录

默认话题列表包含 `front_marker_control` 的 4 个话题，用来分析自动移到码前、到位读码的过程
（进场用了多久、到位时还在不在动、有没有反复进出 `POS_HOLD`）：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/front_marker_control/control_mode` | `std_msgs/String` | 状态机阶段 ALIGN / YAW_SAMPLING / POSE_ALIGNING / POS_HOLD / RETREAT，只在变化时发布（latch） |
| `/front_marker_control/status` | `std_msgs/String` | 状态文本（门控原因、目标、采样统计），变化时和定时发布 |
| `/front_marker_control/position_error` | `geometry_msgs/Vector3Stamped` | 视觉位置误差（机体系） |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 实际发给飞控的 setpoint（20 Hz，门控不通过时为零速度） |

`control_mode` 只在变化时发布，开包前收到的那一条录不进包，所以它在 `latched_topics` 里：
节点平时缓存最新一条，开包时先补写一条当前值，包里第一条就是开包时刻所处的阶段。
控制节点没跑时这些话题没有消息，不影响录包。

## 绕线器状态记录

默认话题列表已加入 `/tether_winch_control/status`，类型为
`tether_winch_control/WinchStatus`。包含档位、距离、径向速度、目标DUTY、
最近发送DUTY/时间和通信错误；这些是控制/发送状态，不是电机实际反馈。

`start_rc_record.sh` 会在 MID360S 环境后追加加载
`/home/orangepi/orange/devel/setup.bash --extend`，保留 Livox 消息环境。
需要板端已经构建 `tether_winch_control` 消息；缺失时会明确告警并跳过此话题。
录制器可先启动，绕线器状态发布者稍后出现时也能订阅；只有录包期间收到的消息会写入bag。
配置修改后需重启录制器生效，不影响原来的RC触发规则，不启动绕线器节点。

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
期望得到 **2 个** bag，各含 pose 和 WinchStatus 消息且都已正常收尾（无残留 `.bag.active`）；
`control_mode` 只在开包前发过（ALIGN，两个包之间又发 POS_HOLD），每个包里应正好有开包时补写的那一条。
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
