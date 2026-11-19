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


## 室内 test3_1 四点 AprilTag 识别

板端使用本仓库和 `junheng-sect/Orange` 的 `main`，后者提供相机、视觉消息、规划器和仓库监控。两个工作空间按各自说明使用 catkin 构建。

从本仓库根目录运行以下命令可复现合成四点测试。脚本使用独立 ROS master 和预览网关，自行生成图像、标签、占据观测及里程计。默认加载 `/home/orangepi/orange/devel/setup.bash`，验证其他检出时可用 `INDOOR_ORANGE_SETUP` 指定已构建的 Orange 环境。

```bash
./scripts/nav_test/tests/run_indoor3_1_isolated.sh
```

默认结果存入 `~/test_data/nav_indoor3_1_validation/<时间>/`。四点产生四张 PNG，样例 CSV 为八行，覆盖 ID 0–5，另列 ID 7。故障和回归模式分别为 `dropout`、`stop`、`photo`，例如 `./scripts/nav_test/tests/run_indoor3_1_isolated.sh "$(mktemp -d)" --mode dropout`。

现场手持预览使用 `./scripts/nav_test_indoor3_1.sh --preview`，实飞入口为 `./scripts/nav_test_indoor3_1.sh`。两者都需要板端已归档的飞扫图 `20261114_101654_indoor_fly_raw_000001` 及匹配的 `prior_grid.bin`。任务复用 test3 的左下、左上、右上、右下四点，每点稳住 1 秒，采样 2 秒。任务目录中的 `apriltag_observations.csv` 按观察点和 ID 记录时间戳、帧数、同步位姿及照片。结束后右下悬停，飞手接管降落。

### 同架次双相机 test3_2

在板端运行 `~/mid360s_ws/scripts/nav_test_indoor3_2.sh`，手持预览加 `--preview`。沿用上述四点、原图拍照与每点 2 秒观察窗口。上方 OV13855 用 1280×960、15 Hz 检测，下方 OV9281 用 USB MJPEG 1280×800、30 Hz 和对应内参。两路均检测 tag36h11。

`apriltag_observations.csv` 每行对应一个点位、一台相机和一个 ID。按 `camera`、`waypoint`、`id` 对照，用 `seen_frames / window_frames` 比较各自的命中率。照片是同一窗口内的 OV13855 全图。任务 JSON 的 `legs[].observation.cameras` 也记录零命中点的帧数、ID 和最大帧间隔，`coverage` 给出各自的 0–5 覆盖情况。任何一路在窗口内出现超过 0.5 秒的间隙，都保存窗口统计并沿用任务断流处理。

## test7：室外全局航点（drone_map 绝对坐标）

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test7.sh
# 自定义航点（可重复 --point，按给出顺序飞）；--hold 改每点保持秒数
./nav_test7.sh --point "0,0,1,0" --point "1.5,0,1.5,90" --hold 5
```

与 test2/4/5/indoor 的区别：**航点直接按 `drone_map` 绝对坐标发**，不再按“当前位姿 + 偏移”算。
任务层 `/drone_nav/execute_route` 本来就只收 `drone_map` 帧，所以这里把绝对点原样交给它，
只为了边界检查和记录反算一次 `drone_odom`。默认用室外链（`20261105_114605_outdoor` + `outdoor.yaml`
`map_frame_outdoor.yaml` + `outdoor_nav.yaml`）。

默认航点（`drone_map`，m，第四个数是偏航角 °），每个点稳定到达后保持 3 s：
`(0,0,1,0)` → `(2,0,1,0)` → `(2,-2,1,0)` → `(0,-2,1,0)` → `(0,0,1,0)`。

- **z 的语义**：`drone_map` 原点 = 建图/重定位启动时机体（雷达 IMU）所在位置，也就是飞机停在地面时的
  机体高度，不是地面海拔；`z=1` 表示机体离地约 1 m（腿和桨保在机体下方约 0.15 m）。`car_bridge` 的
  `map` 帧把地面定在 z=0，比它低 0.151 m（`map_frame_outdoor.yaml`，照室内抄的假设值，未实测）。
- 启动横幅打印当前位置（`drone_map` 与 `drone_odom` 两套）、首点距离、每个点换算后的 odom 坐标；
  首点离当前位置超过 `--max-start-offset`（默认 2 m）直接拒绝，防止拿错地图或原点。
- **首点已在脚下就不发**：离当前位置 <0.2 m 且航向差 ≤5° 时只保持 `--hold` 秒。任务层明确拒绝
  离当前位置 <0.2 m 的新目标（原地转向只在“重复同一终点”时放行），而人工起飞点通常就在首点正下方；
  若 <0.2 m 但航向差 >5°，脚本报错并拒绝，需要先挪开或改点。
- 拒绝条件：相邻航点 <0.2 m、航点换算到 `drone_odom` 后越过边界（`outdoor_nav.yaml` 的
  `[-3,-3,0.4]~[3,3,2.5]` 加 0.30 m 余量，即 x/y 在 ±2.7 m、z 在 0.70~2.20 m）、目标足迹未知/被占
  （`--check-segments` 时查整段直线）、地图变换重置、输入/重定位失效、执行器或网关故障。
- 到点判据由任务层给：位置误差 ≤0.15 m、速度 ≤0.1 m/s、航向误差 ≤5°，连续满足 1 s；到点后保持
  `--hold` 秒再 `prepare` 下一段。
- 每点单独一次 action；记录在 `~/test_data/nav_test/<时间>_test7/goal_test7_*.json`，含
  `goals_map`/`goals_odom`、每段发送/到达/保持时间、终点误差；跳过的段记为 `SKIPPED_AT_POINT`。
- 接管、上锁、故障或 Ctrl+C 立即终止本轮，不续飞剩余航点；不自动解锁、起飞或降落。

预览（导航不发飞控，需手持按目标移动）：

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test7.sh --preview --ev-mode odom

# 离线单测：绝对点、反变换、每点保持、已在点上跳过、拒绝条件（不需要 master/飞控）
cd /home/orangepi/mid360s_ws/scripts/nav_test/tests && python3 -m unittest -v test_nav_global
```

## test8：全局航点 + 段前转向（机头朝前进方向）

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test8.sh
./nav_test8.sh --hold 3          # 每个"前进"段到点后再停 3 s 观察
```

和 test7 同一条路线、同一条室外链，区别只在航向：**每一段都先把机头转到该段的前进方向，再飞这一段**，
也就是"前进 → 原地转 → 前进 → 原地转"。实现上不是改执行器，而是把每个拐点拆成两个目标：

- **前进**：下一拐点的位置 + 该段连线方向作为航向。执行器在平移期间锁住这个航向，所以机头与前进方向一致。
- **原地转向**：与刚完成的终点**完全相同的位置** + 下一段方向，走任务层“重复同一终点”的放行条件
  （位置 1 μm 内、实际误差 ≤0.15 m、速度 ≤0.1 m/s）；执行器按导航配置的 `yaw_rate_deg` 逐 tick 步进。

默认路线（`drone_map` 绝对坐标，与 test7 相同；方括号是自动算出的机头方向）：
`(0,0,1)` → `(2,0,1)[0°]` → `(2,-2,1)[-90°]` → `(0,-2,1)[180°]` → `(0,0,1)[+90°]`。
第 4 个字段（航向）在 test8 里被忽略，只用位置。

- 转向速率来自导航配置：test8 默认 `outdoor_nav_yaw45.yaml`（`yaw_rate_deg: 45`，**转 90° 约 2 s**），
  其余限速/包络/边界/起飞高度与 `outdoor_nav.yaml` 完全一致；test7 仍用 30°/s。
- **第一段转不了**：任务层不接受首目标的原地转向，所以起飞后机头要大致对着首段方向
  （默认路线是 `map +x`，即建图时的机头方向）；偏了只会让第一段轻微侧飞，第二段起就完全对齐。
- 起飞前仍要升到 **0.70 m 以上**（边界 z_min 0.40 + 余量 0.30），否则发目标前就会被预检拒绝。
- 默认不保持（`--hold 0`）：转向段转完立刻前进；`--hold S` 只对“前进”段生效。
- 记录在 `~/test_data/nav_test/<时间>_test8/goal_test8_*.json`，`legs` 里每个目标的 `direction` 标明是
  "前进 #i→#j，机头 X°"还是"原地转向到 X°（±Δ°）"，并带发送、到达、保持时间与终点误差。

**原地转向在本项目是第一次上实飞**：隔离测试里（test5 的 90° 原地转向）转向漂移 7.7 mm、无异常，
但实飞的推力、地面效应和 LIO/PX4 航向残差还没验证过；第一次跑建议低空、空旷、留足余量，并盯住转向时的位置漂移。

```bash
# 预览（导航不发飞控，手持按目标移动/转向）
cd /home/orangepi/mid360s_ws/scripts
./nav_test8.sh --preview --ev-mode odom

# 离线单测：前进/转向展开、航向=前进方向、转向段不保持、共线路线不转向、180° 掉头
cd /home/orangepi/mid360s_ws/scripts/nav_test/tests && python3 -m unittest -v test_nav_turn
```

## test9：竖直矩形（1 m ↔ 3 m）

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test9.sh                    # 默认路线，每点停 3 s
./nav_test9.sh --hold 0           # 到点就飞，不停
./nav_test9.sh --point "2,0,1" --point "2,-2,2"   # 自定义（3 个数=只管位置）
```

默认路线在 **x=2 平面上的 2 m × 2 m 竖直矩形**，全程机头保持进入本轮时的朝向：
`(2,0,1)` → `(2,-2,1)` → `(2,-2,3)` → `(2,0,3)` → `(2,0,1)`
（先在 1 m 沿 −y 飞 2 m → 竖直爬到 3 m → 沿 +y 飞 2 m → 竖直降回 1 m）。

- **必须用 `nav_test/outdoor_nav_z3.yaml`**（脚本默认）：它只把 z 上界从 2.5 抬到 3.5，配合
  `goal_boundary_margin 0.30` 允许航点到 **3.20 m**；用默认 `outdoor_nav.yaml`（上界 2.20 m）时
  z=3 的两个点会在发目标前被边界预检拒掉。竖直限速仍是 `max_vel_z 0.3`，2 m 爬升约 8 s。
- `--point` 支持 3 个数（`x,y,z`，航向保持）或 4 个数（`x,y,z,yaw_deg`，给目标航向），test7 那种写法照样能用。
- `--max-start-offset` 默认 **3 m**（首点 (2,0,1) 离起飞点约 2 m，用默认 2 m 会贴边界）。
- 起飞前升到 0.70 m 以上；到点判据同其它测试（位置 ≤0.15 m、速度 ≤0.1 m/s、航向 ≤5° 连续 1 s）。
- 记录在 `~/test_data/nav_test/<时间>_test9/goal_test9_*.json`（`--record-prefix test9`；
  json 里的 `test` 字段仍是 test7——两者共用同一个 runner）。
- **风险**：3 m 高度上雷达往上只看到 +52°，头顶是未知格，靠 `allow_unknown_above: true` 当空闲放行；
  室外要留意头顶树枝/电线（看不见就不会被发现），并确认 3 m 附近没有障碍。竖直爬升只靠 LIO/PX4 高度，
  本机没装测距仪。

```bash
# 预览（导航不发飞控，手持按目标移动）
cd /home/orangepi/mid360s_ws/scripts
./nav_test9.sh --preview --ev-mode odom

# 离线单测（含"test9 路线需要 z3 配置""省略航向时保持机头"两项）
cd /home/orangepi/mid360s_ws/scripts/nav_test/tests && python3 -m unittest -v test_nav_global
```

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

## test10：室外完整起降与三轴综合测试（仅一圈，预计 70–90 s）

**人工解锁 → 自动起飞 → 六段航线 → OFFBOARD 慢速下降（0.1 m/s）→ 近地速度下压 → 确认着地后普通上锁**。不自动解锁，不切 OFFBOARD 重跑，不抢回飞手接管的模式，也不切 `AUTO.LAND`（PX4 自带降落速度由 `MPC_LAND_SPEED` 决定，实测 0.4 m/s，太快）。沿用 test8 的 `outdoor_nav_yaw45.yaml`，不改变 test7/8/9 的路线、PX4 参数或包络。

### 路线

`drone_map` **绝对**坐标，m；yaw 为度，0° 朝建图时 +x，正方向向左。z=0 是原点处着地机体高度，并非独立地面海拔测量。起点须在 map 原点附近（水平 ≤0.30 m，z ±0.10 m），机头偏离 map 0° ≤10°；不会把路线偷偷平移或旋转。

|阶段|目标 `(x,y,z,yaw°)`|动作|额外保持|
|---|---|---|---|
|起飞|原地升到约 1 m|人工解锁后等 3 s，自动切 OFFBOARD 起飞|实测稳定 2 s|
|1|`(2,0,1,-90)`|前进 2 m，到点后右转 90°|0 s|
|2|`(2,-2,1,-90)`|机头朝前沿 −y 前进 2 m|0 s|
|3|`(2,-2,2,0)`|原地上升 1 m，到点后左转 90°|1 s|
|4|`(0,-2,2,0)`|保持机头后退 2 m|0 s|
|5|`(0,-2,1,0)`|原地下降 1 m|1 s|
|6|`(0,0,1,0)`|保持机头向左侧飞 2 m，返回|2 s|
|降落|返回点原地|`/drone_nav/land`：OFFBOARD 里按 0.1 m/s 下降，近地低/慢连续3 s后以0.35 m/s Z速度下压最多5 s；新鲜PX4着地后普通上锁|等飞控确认|

水平 8 m、范围 2×2 m、航线高度 1–2 m。yaw **45°/s**，上升竖直 **0.3 m/s、0.3 m/s²**，水平限速/加速度均为 1.0；降落主段 **0.1 m/s**、近地稳定后的短时下压 **0.35 m/s**（慢降网关参数 `~land_vel`，默认 0.1，允许 0~0.5）；不扩大边界。test10 会检查这些配置值，改用不匹配的 `--nav-config` 将拒绝，不能用它偷偷提高速度或起飞高度。

**先移动，到点稳定后再转 yaw，不是边飞边转。** 两次转向嵌入移动目标的终点航向，省去额外转向 action 和 prepare。到点判据仍为位置 ≤0.15 m、速度 ≤0.1 m/s、航向 ≤5°，连续 1 s。预计解锁至落地上锁 70–90 s（不含每段最多额外 10 s 的目标观测等待；1 m 以 0.1 m/s 下降约 10 s，加 prepare→READY 1 s 和自动上锁 2 s），仅供安排实验；风况、重规划、等待和接管会改变用时。

### 板端命令与现场操作

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test10.sh --dry-run             # 只打印，不初始化 ROS、不启动链路、不写运行记录
./nav_test10.sh --delay 3             # 实飞：必须有飞手在场，脚本不自动解锁
```

1. 对应室外地图 `20261105_114605_outdoor` 场地，飞机原点附近上锁，机头朝 map +x。启动时已经解锁或在空中不触发。等待「导航就绪」和「预检通过」。 启动时若仍是 ALTCTL 等模式，会保持地面上锁等待，明确提示遥控器切到 POSCTL（定点）；模式及输入连续稳定 2 s 后自动继续包络预检，无需重启。启动等待总共最多 60 s；等待期间任何一次解锁都会取消本轮，即使随后又上锁也不继续。必须等「预检通过」后再解锁。
   导航 READY 只代表输入新鲜。地面严格检查起飞终点：未知/地图未更新时每 0.5 s 重查，最多 10 s；后续目标若仅为未知，记录为 deferred，留到该段出发前核验，不在地面一直等整条路线的观测。后续已知占据/越界仍在地面拒绝；预检期间解锁、移动超过 5 cm、状态故障或 RPC 超时立即退出。看到「起飞预检通过」才人工解锁。flight JSON 的 preflight_checks/deferred_ground_targets 保留原始 valid=false 与原因，绝不把 deferred 当成已知空闲。
   起飞稳定后，每段出发前严格检查该目标包络：未知则保持当前悬停点，每 0.5 s 重查、最多 10 s，期间不发本段目标；仍未知则终止本轮，保持点位并由飞手接管。占据/越界立即终止。通过后原有发目标前复查、EGO 实际轨迹和执行器剩余轨迹检查继续生效。leg_coverage_checks 记录每段核验结果。等待期间模式/定位/数据流守卫继续工作，悬停偏移超过 0.15 m 或速度超过 0.15 m/s 会终止。
2. **通道 6 开录包，然后 POSCTL 地面人工解锁**。脚本只接受本轮待命期间看到的解锁边沿，3 s 后切 OFFBOARD 自动起飞。不必再人工起飞或切 OFFBOARD。
3. `takeoff=DONE` **不等于实测到高**：实测相对高度和 map 高度均在 1 m ±0.10 m、速度 ≤0.1 m/s、飞控报在空中，连续 2 s，才开始平移。变换在解锁前冻结，全程监视，不能起飞后重取而掩盖重置。
4. 6 段全部成功且返回稳定，降落预检通过，才重新 `prepare` 并调用 `/drone_nav/land`。保持OFFBOARD与XY/yaw，上段0.1 m/s；实测高度≤地面参考上方15 cm、|vz|≤0.20 m/s、水平速度≤0.25 m/s连续3 s后进入PRESSING，以0.35 m/s垂直速度下压（忽略Z位置，最多5 s），不在人工高度终点零速悬停。原地面下10 cm位置字段限幅保留；到位置限幅时继续慢降，最多再等2 s进入接地/下压条件；速度控制期间位置字段仅诊断，实测达到地面下10 cm则停止。只有新鲜PX4着地确认才TOUCHED并冻结目标、速度归零；随后每秒最多一次普通disarm(value=False)，最多5 s，确认实际上锁后DONE。低高度/低速度或服务ACK均不算成功；始终不报着地、着地不上锁、模式接管、反馈/定位异常分别终止。整体降落监视40 s仍保留，不切AUTO.LAND、不自动解锁、不强制上锁。
5. 着地上锁后**先拨回通道 6 停录包，再走近拿飞机，最后 Ctrl+C 收尾**。链路不会自动拆掉，常驻 roscore/MAVROS 不起停；重新实验需要明确重新启动脚本，不因模式切换重跑。

人员建议距航线至少 4 m，落点平整无障碍，上方无树枝/电线。`allow_unknown_above=true` 仍然**不能保证避让头顶障碍**。全程飞手可用模式开关或推杆接管；如果模式开关本来仍停在定点，需先拨走再拨回。**Ctrl+C 不是降落操作，也不应空中强制上锁。**

接管、断流、地图重置、异常网关/执行器故障及超时永久结束本轮，即使状态恢复也不续飞，不另行返航或自动降落。起飞/航段最多 20 s；慢速降落监视最多 40 s（1 m 约 10 s，加准备与上锁）。下降由网关独占设定值流：退出 OFFBOARD、网关故障或 READY 丢失即停止下降并原地减速保持，不续降、不补发；落地上锁后 PX4 报 const_pos 属于地面正常状态。故障后定位链路保留，须飞手安全处置。

### 记录与预览

`~/test_data/nav_test/<时间>_test10/` 中保存 RC bag、`nav_test.log`、`goal_test10_*.json` 和 `flight_test10_*.json`。完整流程记录保存各阶段时间、地图变换、状态、结束原因；goal 的 `legs[].phases` 按实际位置/航向与执行器状态标记 `MOTION / TURN / SETTLING / HOLD`，分析时不要把第 1/3 段整段都算成纯平移或纯 yaw。阶段切分是遥测判别，精确角速度/漂移仍用 bag/ulg 对齐复核。

```bash
cd /home/orangepi/mid360s_ws/scripts
./nav_test10.sh --preview
```

预览仅测试空中六段，不测试自动起飞/降落，不创建起降或切模式服务代理；要求网关 `real_output=false`。预览膨胀与实飞相同（0.40/0.30 m），手持从约 `(0,0,1)` 开始按目标移动、转向。预览成功不能当作实飞或自动起降验收。

### 离线验证

```bash
source /opt/ros/noetic/setup.bash
source /home/orangepi/mid360s_ws/devel/setup.bash
source /home/orangepi/orange/devel/setup.bash
cd /home/orangepi/mid360s_ws/scripts/nav_test/tests
PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -p 'test_nav_*.py'
./run_outdoor_full_isolated.sh
```

隔离集成使用全新 localhost:11443 Master、模拟 FCU/导航服务/action，拒绝连接已占用端口，不启动 fusion/EGO/MAVROS，不创建解锁服务或飞控设定值发布者。覆盖正常全流程、DONE 但高度未到、错误起点/yaw、启动已解锁、接管/输入故障恢复、地图重置、状态断流、航段超时、慢速降落被拒、着地未上锁。故障场景只在测试进程内缩短部分截止时间，正常流程沿用正式稳定门槛。它验证时序、守卫和 ROS 投递，不证明真实飞控、规划动力学或地图安全。

现场验收仍需一轮完整起降、六段成功、双向 yaw 正确、无非预期模式切换或 failsafe；结合 bag/ulg 评估水平/竖直误差、转向漂移和落点偏差。融合 odom 不是独立位置/高度真值。

ROS 服务单次等待上限 2 s，并受起飞/航段/降落阶段的剩余时间约束；等待期间安全守卫继续工作，超时或通信异常立即结束本轮、不重发不确定的请求。ROS 不能撤回已投递的请求：若切模式/起飞请求响应卡住，飞控仍可能已执行或稍后执行该**已发出的一次请求**，飞手须检查实际模式并接管；脚本不会自动补发或续飞。隔离验证另覆盖服务响应卡住的情况。

### 降落末段验证（2026-11-13）

仍保留40 s总降落超时、不强制上锁、不自动重试降落。着地后普通上锁允许按1 s间隔请求（最多5 s），上锁RPC异步执行，不能阻断设定值流。flight JSON 的 landing_states/stop_state 增记Z速度模式、实际指令vz和上锁请求次数/结果；超时区分未检测到着地与着地后未上锁。默认bag保留 /mavros/extended_state 和 /mavros/estimator_status。

后续实飞20261113_164142表明，上一版“位置到地面下10 cm后速度归零并等2 s”仍未得到PX4着地确认。因此旧修复只作为历史验证，不再代表当前末段逻辑。现参考 [land_with_tracking_v2](https://github.com/junheng-sect/ROS2PUBLIC/blob/rasip_pi/src/land_with_tracking_v2/land_with_tracking_v2/land_with_tracking_v2_node.py) 的持续速度下压与普通disarm；本项目仍仅允许在新鲜PX4着地后请求上锁。

真实网关专项隔离验证：`bash /home/orangepi/orange/src/drone_navigation/test/run_landing_isolated.sh /tmp/landing_validation`（仅模拟FCU与预览输出）。本次bag固定输入回归工具为同目录 `replay_landing.py`。14:00旧批次缺少原始着地/估计器话题；16:41新批次已具备这些原始输入。当前使用16:41批次的真实着地/估计器/模式/位置记录验证：不误报TOUCHED、不在空中请求上锁，并能进入Z速度下压。固定历史输入不会响应新控制，不能证明实机会落地。当前完整证据见 `/home/orangepi/test_data/nav10_validation/20261113_velocity_press_fix/`；旧版验证留作历史。

### 新建带图任务的 DLL 旁路

新脚本使用唯一的 `TEST_NAME`，载入 `nav_test/common.sh` 并调用 `nav_test_main "$@"`，即可继承通道 6 的轻量对照录包和 DLL 旁路。室内 test3_1 已走这个入口。已有 test1–11、室内 test1–4 保留各自原有配置；室外 test12 继续走专用入口。

用 `--show-dll-config` 可以在启动前核对当前任务的默认策略；命令只打印配置。导航就绪后，任务用本次重定位地图启动 DLL。已知大室外图选用经同源哈希校验的裁图。网页同时保留原导航主位姿与 DLL 对照栏。目录中的 `dll.log`、`dll_observer.json` 和配置副本用于复盘。对照量是同一扫描时刻的 DLL 减 FAST-LIO2，不写入生产 TF。

默认按原有设置录包，并增加两路地图位姿、校正、状态与会话信息。需要雷达原始数据以便重跑或静止检查点 ICP 时，在本次命令末尾加 `--record-raw-lidar`；只用原定位可加 `--no-dll`。通用录包导出：`python3 ~/orange/tools/warehouse_monitor/export_dll_drift.py "$bag" "${bag%.bag}_drift"`（先 source 板端 mid360s_ws 环境）。四点观察的短停可以作同帧差值分析；ICP 工具只为满足其静止时长和几何质量条件的窗口出参考值。
