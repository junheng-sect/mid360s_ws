# 导航实飞测试

## test indoor 1：室内前进 1 m

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test_indoor1.sh --ev-mode odom
```

使用旧室内地图 `/home/orangepi/test_data/mapping/20261021_110311_indoor`、
`drone_relocalization/config/indoor.yaml` 和 `car_bridge/config/map_frame.yaml`。
飞机静置完成两个窗口的重定位确认，等“导航就绪”后，通道 6 开始录包，人工定点起飞并悬停。
低空测试可将导航 z 稳定在约 0.5～0.6 m（最低允许 0.40 m；此值不是测距离地高度）。
手动切入 OFFBOARD，原地保持 3 s 后冻结当前位置、高度与水平机头方向，再沿此方向前进 1 m。
保持高度与航向，到点稳定后记录 5 s 保持，随后继续悬停。

**每次重新切入 OFFBOARD 都从当时位置再次向前 1 m，不自动返回。**
启动时已在 OFFBOARD 不触发；倒计时内切出取消；飞行或取消期间的模式边沿不会意外触发下一轮。
接管、输入失效、重定位失效、TF 重置或 action 失败均终止本轮。
结束时切回定点、人工降落、上锁，再 Ctrl+C；SSH 断开后 `tmux attach -t nav` 恢复终端。

- 独立 `indoor_nav.yaml`：三维速度/加速度上限 0.5 m/s、0.5 m/s²；EGO 单轴上限约 0.289。
- 机体半包络 0.27/0.27/0.15 m，膨胀 0.4/0.3 m，`allow_unknown_above=false`。
  旧地图用于重定位；实时点云用于路径检查，整段直线和实际样条都要通过检查。
  未知、占据和范围外均拒绝，不因室内盲区而放宽检查。
- `drone_odom` 范围 `[-3,-3,0.1]～[3,3,2.5] m`，起点/终点距边界至少 0.30 m，
  即起点高度需在 0.40～2.20 m。低空下界单独下调，机体包络、碰撞和未知区域检查保持开启。
  多次前进接近边界时拒绝本轮目标；高度不是测距离地高度。
- 到点：位置误差 ≤0.15 m、速度 ≤0.1 m/s、航向误差 ≤5°，连续满足 1 s。
- 记录在 `~/test_data/nav_test/<时间>_test_indoor1/`：`test_indoor1_*.bag`、
  `goal_test_indoor1_*.json`、重定位结果与各节点日志。可用下文 `analyze_sequence.py` 分析 bag 和目标 JSON。

预览命令如下，导航不会向飞控发送设定值；仍需真实定位/重定位输入，按回车后需手持移动到点。
**室内预览沿用实飞包络**，不会套用旧室外预览的小膨胀。

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test_indoor1.sh --preview --ev-mode odom

# 独立 localhost Master + 合成观测，验证 EGO/任务层/执行器/预览网关，不连接常驻 Master
bash /home/orangepi/mid360s_ws/scripts/nav_test/tests/run_indoor1_isolated.sh
```

实现验证与实飞验收边界见 `INDOOR1_VALIDATION.md`。以上入口均需先结束其他导航测试。

## test indoor 2：固定机头的 0.5 m 方形

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test_indoor2.sh --ev-mode odom
```

沿用室内1的地图、两次重定位确认、低空配置、限速和包络。等“导航就绪”后通道 6 开始录包，
人工定点起飞悬停，手动切入 OFFBOARD，保持 3 s 后冻结本轮起点、高度和航向。
全程机头朝向不变，依次前 0.5 m、右 0.5 m、后 0.5 m、左 0.5 m 回起点。
**每个点稳定到达后再保持 2 s**，包括最后返回起点；一圈后继续悬停，切回定点再切 OFFBOARD 才重跑。
启动时已经 OFFBOARD 不触发，接管或故障立即终止本圈，不续飞剩余航段。

四个航点相对本轮起始机头方向一次算好（前为 x 正、左为 y 正）：
`(0.5,0)` → `(0.5,-0.5)` → `(0,-0.5)` → `(0,0)`。
出发前预检整圈起点和航点的边界余量，每段出发前检查当前位置到目标的整段直线，实际样条继续检查。
未知、占据或越界会拒绝本段；本测试不放宽室内未知区域或机体检查。

- 最低允许导航 z 仍为 0.40 m，低空测试可在导航 z 约 0.5～0.6 m 悬停；这是导航坐标，不是测距离地高度。
- 到点标准仍为位置误差 ≤0.15 m、速度 ≤0.1 m/s、航向误差 ≤5°，连续满足 1 s，然后再保持 2 s。
- 记录目录 `~/test_data/nav_test/<时间>_test_indoor2/`，bag 为 `test_indoor2_*.bag`，
  目标为 `goal_test_indoor2_*.json`，其中 `side_m=0.5`、`hold_s=2.0`；可用 `analyze_sequence.py` 分析。
- 室内1仍前进 1 m 并保持 5 s；旧室外 test2 仍为 1 m 方形、每点保持 5 s。

预览沿用相同室内包络，仍需手持/模拟状态按目标移动到点：

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test_indoor2.sh --preview --ev-mode odom

# 独立 localhost:11442，合成导航 z=0.50 m，不连接实机 Master 或发送飞控设定值
bash /home/orangepi/mid360s_ws/scripts/nav_test/tests/run_indoor2_isolated.sh
```

现场结束时人工切回定点、降落、上锁，再 Ctrl+C。验证记录见 `INDOOR2_VALIDATION.md`。

## 其他航段测试

所有入口都先运行定位、重定位和导航；等待“导航就绪”以后，人工定点起飞并悬停。
通道 6 控制录包。切入 OFFBOARD 后等待 3 s，冻结本轮位置、水平机头方向和高度。
启动时已经处于 OFFBOARD 不触发；一轮结束后必须切回定点再切 OFFBOARD 才重跑。

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test4.sh --ev-mode odom
# test4 收尾以后再运行 test5，二者不能同时运行
./nav_test5.sh --ev-mode odom
```

- test4：前进 2 m、返回、前进 4 m、返回；各点保持 5 s，高度和航向不变。
  4 m 单程只发一个目标，由 EGO 在 3 m 规划视距内生成中间轨迹并连续重规划。
- test5：升高 0.5 m、降回；前进 1 m 同时升高 0.5 m、斜着返回；在起点依次左转 90°、180°、270°、360°。
  平移时保持起始航向，原地转向限 10°/s。各点稳定到达以后再保持 5 s。
- 到点：位置误差 ≤0.15 m，速度 ≤0.1 m/s，yaw 误差 ≤5°，连续满足 1 s。
- test4 默认 `outdoor_nav_long.yaml`（水平 ±5 m）；test5 和旧测试默认 `outdoor_nav.yaml`（±3 m）。
  两者 z 范围均为 drone_odom 0.4～2.5 m，航点与起点需要距各边界至少 0.30 m。
  因此 test5 的 z₀ 需位于 0.70～1.70 m。高度是控制参考点坐标，不是测距离地高度。
- test2、test4、test5 会在首段发送前检查整轮航点的边界余量。任务层再次检查提交的所有航点。
  `goal_boundary_margin` 为有限非负数，旧配置缺省 0。
- test5 每段出发前检查当前位置到目标的路径。未知格、占据和范围外均拒绝；不放宽实飞包络。
  检查通过只说明当时的直线路径可用，实际样条还会由执行器持续检查。
- 原地转向只能承接上一段已经完成的终点，位置相同（1 μm 数值容差）、实际误差 ≤0.15 m 且速度 ≤0.1 m/s。
  新短距离移动不因此放行；原地转向仍使用 EGO 新轨迹代次、执行器地图租约和网关保护。
  任务层明确拒绝不足 0.2 m 且不满足上述锚点条件的目标，避免到点容差在没有轨迹时误报成功。
  EGO 用现有 arrived/cancel 通知同步正常到点，兼容两种到达顺序；执行器可暂存先于目标回调到达的有效样条，
  仅在 0.3 s 内、起始时间戳匹配本次目标时采用，并继续执行地图检查。取消会清除暂存。

test4 前需清空整个飞行范围，包括停放车辆。起飞点应靠近启动 fusion 时的原点；边界预检拒绝时先看输出坐标与余量。
任意接管或故障都停止本轮，不会续飞剩余航段；不自动清除真正的执行器故障，必要时落地收尾后重启测试。

预览入口如下（定位和地图仍需实物/模拟输入；脚本不会自动模拟飞机移动）：

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test4.sh --preview --ev-mode odom
./nav_test5.sh --preview --ev-mode odom
```

旧室外预览沿用较小膨胀，供手持操作；室内入口和下面独立验收均使用实飞膨胀 0.4/0.3 m、机体半包络 0.27/0.27/0.15 m。

接管时用遥控器切回定点。正常结束后人工降落、上锁，再按 Ctrl+C 收尾。Ctrl+C 可停止任务发送，但飞机仍解锁时会保留外部定位链路。
SSH 断开可用 `tmux attach -t nav` 返回；不要在另一终端重复启动测试。

## 离线验证和分析

在板端运行，均使用独立 localhost Master，不连接常驻 Master、不发布真实 MAVROS 设定值：

```bash
bash /home/orangepi/mid360s_ws/scripts/nav_test/tests/run_sequence_isolated.sh
bash /home/orangepi/mid360s_ws/scripts/nav_test/tests/run_climb_replay.sh \
  /home/orangepi/test_data/nav_test/20261107_134510_test2/test2_000001.bag \
  /home/orangepi/test_data/nav45_validation/climb_replay
```

每轮记录位于 `/home/orangepi/test_data/nav_test/<时间>_test4/` 或 `_test5/`，包括日志、RC 触发 bag、`goal_test4_*.json` / `goal_test5_*.json`。
目标 JSON 保存 drone_odom/drone_map 坐标、航向、每段发送/到达时间、终点位置/高度误差及保持区间。

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/orange/devel/setup.bash
python3 /home/orangepi/mid360s_ws/scripts/nav_test/analyze_sequence.py \
  /绝对路径/test4_000001.bag /绝对路径/goal_test4_时间.json --out /绝对路径/analysis
```

输出 `summary.json`、`legs.csv`：按执行器接受的轨迹 id 统计重规划（不含首次），同一时刻对比换轨 Δp/Δv/Δa，记录峰值速度/加速度、终点误差和耗时。
航向段统计指令转向时间、位置漂移、保持误差和 LIO/PX4 航向残差；航向残差使用任务前两秒估算静态坐标偏置，不能替代网关的精确同步对齐。
同时报告竖直跟踪 RMS/峰值和实际相邻指令的连续性余量。执行器的连续性阈值是 4 cm 加上时间间隔允许的位移，
不能把同一时刻两条样条的差值直接等同于相邻指令跳变。
缺失数据输出 null。推力、电机输出和物理爬升跟踪须结合匹配的 ULog。

爬升回放使用完整起飞过程的历史观测，不向未飞过的高度补造观测；历史未知格拒绝须保留为限制。
合成理想跟踪、历史观测覆盖和真实飞行验收分别报告。
