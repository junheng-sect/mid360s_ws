# mid360s_calib

MID-360S 与飞控外参标定（iKalibr）的数据采集包：遥控器通道 6 触发录**标定专用** bag。
平时飞行仍用 `scripts/fusion.sh` + `scripts/rc_record`，不受本包影响。

## 用法

```bash
cd /home/orangepi/mid360s_ws && source devel/setup.bash
roslaunch mid360s_calib calib_record.launch
```

启动 MAVROS（`/dev/ttyS3:921600`）+ MID-360S 驱动（CustomMsg）+ 标定录包节点。
**不要和 `fusion.sh` / `start_handheld_test.sh` 同时运行**：同一 ROS Master 上同名节点会把正在
运行的 `/mavros` 踢掉，飞控串口也会冲突。

| launch 参数 | 默认 | 说明 |
|---|---|---|
| `bag_dir` | `/home/orangepi/test_data/bags/calib` | 包目录 |
| `bag_prefix` | `calib` | 包名 `calib_000001.bag`，每拨一次开关新增一个序号 |
| `imu_raw_rate` | `400` | 录包期间请求的飞控 IMU 频率；0 = 不改（默认 50 Hz） |
| `fastlio` | `false` | `true` 时额外起 FAST-LIO，只用来现场确认雷达正常（不起桥接、不送飞控） |
| `fcu_url` 等 | 同 `fusion.launch` | MAVROS 连接参数 |

## 录什么

| 话题 | 类型 | iKalibr 类型 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | `LIVOX_CUSTOM` |
| `/livox/imu` | `sensor_msgs/Imu`（加速度单位 g） | `SENSOR_IMU_G` |
| `/mavros/imu/data_raw` | `sensor_msgs/Imu`（m/s²、FLU） | `SENSOR_IMU`，作 `ReferIMU` |
| `/mavros/timesync_status` | `mavros_msgs/TimesyncStatus` | 不参与标定，事后核对时间戳 |

约 5 MB/s（主要是原始点云）。

## 飞控 IMU 提频

PX4 默认只以 50 Hz 发 HIGHRES_IMU。节点在**开包后**调用 `/mavros/set_message_interval`
请求消息 105 @ 400 Hz，**关包后**恢复默认；录包中飞控重启（MAVROS 重连）会重新请求。
板端实测：请求 400 Hz 得约 190 Hz（`IMU_INTEG_RATE=200` 封顶，偶尔漏一个样本）；请求 200 Hz
反而因混叠只有约 158 Hz。该设置是易失的，飞控重启即恢复默认。
空闲和正常退出时飞控已是默认频率；只有录包中直接 Ctrl+C 才可能来不及恢复（会告警），
此时重启飞控即可。

## 采集步骤

1. 桨叶拆下，雷达、飞控、遥控器上电，启动 launch，确认日志「标定录包就绪」。
   若提示「仍未收到 /mavros/rc/in」，说明遥控器没开或没绑定，此时拨开关不会录包。
2. 通道 6 拨到触发位（PWM 大的一侧）开始录：先静止约 5 s，再手持整机边绕圈边做 8 字晃动，
   三个轴都转到、各方向都有平移，不要太快也不要太慢；总长 60–120 s，周围环境结构尽量丰富。
3. 拨回去 1 s 后关包，日志打印**体检摘要**（各话题条数与平均频率）；话题缺失或飞控 IMU
   低于 150 Hz 会告警，该包需要重录。
4. 状态：`rostopic echo /calib_record/status`、`/calib_record/bag_path`。

## 文件

| 文件 | 说明 |
|---|---|
| `scripts/calib_bag_recorder.py` | 录包节点：RC 触发、写包、录包期间提频、关包体检 |
| `scripts/rc_record_policy.py` | 开关策略（去抖 / 尾巴 / 启动时开关已在触发位不开包），原样复制自 `scripts/rc_record` |
| `scripts/sequence_allocator.py` | 六位序号分配（崩溃不复用编号），原样复制自 `scripts/rc_record` |
| `config/calib_record.yaml` | 节点参数（触发阈值、输出、提频） |
| `launch/calib_record.launch` | 一键启动 |

## 测试

```bash
cd /home/orangepi/mid360s_ws
catkin build mid360s_calib --catkin-make-args run_tests && catkin_test_results build/mid360s_calib
src/mid360s_calib/test/calib_record_e2e.sh      # 隔离 Master + 假数据端到端（端口默认 11379）
```

端到端脚本翻转一次假 RC，期望只生成 1 个包、只含上面 4 个话题（干扰话题 `/Odometry` 不进包），
假的 `set_message_interval` 服务依次收到 400 与 0，节点日志里有体检摘要。
