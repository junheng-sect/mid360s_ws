# 本机 Docker RViz

本目录随 mid360s_ws 管理显示脚本、Dockerfile、RViz/launch 配置与面板测量工具。
用于 Ubuntu 桌面 X11 + NVIDIA Docker 环境，不在 Orange Pi 板端运行。

在仓库根目录调用（仓库可放在任意路径，用户名不必相同）：

```bash
./scripts/run_rviz.sh
./scripts/run_rviz.sh --detach
./scripts/run_rviz.sh --help
./scripts/run_rviz.sh --reloc
```

`--reloc` 打开重定位对照视图（`rviz/drone_reloc.rviz`，个人副本 `home/ros/.rviz/drone_reloc.rviz`，规则同下）：
Fixed Frame `map`，彩色先验地图 + 白色实时点云，三维里有 map（充电桩原点）、车 `car_base`、无人机 `drone_base` 三组坐标轴（都是板端真实 TF）；
左栏下方是摄像头预览，三维视图下面是二维车图 `Car map`。需要板端运行 `drone_relocalization` 的 `relocalize.launch`（带 `car_bridge`），
板端不跑显示节点：`--reloc` 会在容器里用 `launch/display_reloc.launch` 同时起 RViz 和显示辅助节点 `local_view/reloc_display.py`
（二维车图、标签、状态文字；车栅格图直接从车端 rosbridge `192.168.31.231:9090` 取，只订阅 `/map`）。`local_view` 不用编译，
启动时加进 `ROS_PACKAGE_PATH`；它要 `python3-websocket`，镜像已带上（改过 Dockerfile 后先 `./scripts/build_image.sh`）。
说明见 `drone_relocalization` 包 README「RViz 对照显示（本机）」。两个 Image 面板都走 compressed，
启动时 RViz 会报一条 `Tried to advertise a service that is already advertised ... /rviz/compressed/set_parameters`，是两个面板共用 compressed 参数服务，不影响显示。

## 无人机状态桥（网页无人机面板）

```bash
./scripts/run_drone_status.sh          # 后台起容器 drone_status；--fg 前台看日志，--stop 停
curl -s http://127.0.0.1:8091/status   # 汇总好的 JSON
```

`local_view/drone_status.py` 在容器里只读订阅板端常驻 MAVROS 的低频话题（state、battery、sys_status、extended_state、
estimator_status、statustext、送给飞控的视觉位姿），加上每 2 s 问一次板端 master（fusion 核心节点在不在、MAVROS 有没有重启），
汇总成一份 JSON，带“可以起飞 / 不可起飞”的判断。板端不加进程，只多一个订阅者；本节点不发布、不调服务、不写参数。
页面是 orange 仓库的 `tools/car_web`（顶栏“无人机”“机电量”，点“无人机”展开），默认取 `http://<打开页面的主机>:8091/status`，
手机经台式机 IP 打开也能用。镜像里没有 mavros_msgs，节点按发布者连接头里的消息定义现场解析，不用重建镜像。
板端没开时照样起，页面显示“连不上板端”；板端重启（master 换了）节点自己重来。本机换网后 ROS_IP 变了，要重跑一次。

## 两台电脑共用一份仓库

公共脚本、Dockerfile、默认 RViz 配置正常提交和同步；每台电脑单独配置驱动、Docker 和 NVIDIA runtime。
RTX 3060 Laptop 与台式 NVIDIA 显卡共用该入口，建议使用 Xorg 桌面。

在仓库根目录执行一次（已有 local.env 时不要覆盖）：

```bash
cp -n docker/ros-noetic-rviz/local.env.example docker/ros-noetic-rviz/local.env
```

按需要编辑 `local.env` 中的 `RVIZ_ROS_MASTER`、`RVIZ_ROS_IP`、`RVIZ_CONTAINER_NAME`、
`RVIZ_IMAGE_TAG`、`RVIZ_CONFIG` 及左栏预算。它是可信的 Bash 配置文件，不进 Git；可以不创建，默认即可使用。
优先级为：命令行参数 > local.env 的 RVIZ_* 设置 > 原有环境变量（ROS_MASTER_URI/IMAGE_TAG/CONTAINER_NAME）> 内置默认值。
`RVIZ_ROS_IP` 留空自动识别，不读取可能由其他 ROS 环境设置的 ROS_IP。

首次普通显示启动将公共 `rviz/fastlio_mid360s.rviz` 复制到本机的
`home/ros/.rviz/fastlio_mid360s.rviz`。以后打开该副本，在 RViz 中保存即可保留本机布局和显示设置；
台式机与笔记本各有一份，不互相覆盖。个人副本存在时不会被新启动或 git pull 重置。
要采用更新后的公共模板，先关闭 RViz，备份/移走个人副本后重新启动。

容器中的 `/opt/rviz-ws` 为只读，个人 `/home/ros` 和 `/logs` 可写，避免 RViz 保存动作修改公共配置。
`--config` / `RVIZ_CONFIG` 仍接受容器内路径；想保存自定义配置，请放在 `/home/ros` 下。
若显式打开 `/opt/rviz-ws/rviz/fastlio_mid360s.rviz`，只能读取，需“另存为”到个人目录。
`--shell`、`--cmd`、`--dry-run` 不初始化个人布局。

默认使用已有镜像 `ros-noetic-rviz:noetic`；首次安装需 Docker、NVIDIA Container Toolkit、
`xauth`、`xhost`、`xdpyinfo`、`xdotool`、`xwininfo`、`xprop` 和 Python 3。
若本机没有镜像，在本目录执行 `./scripts/build_image.sh`。不需要编译板端 ROS 工作空间。

默认 ROS Master 为板内地址 `http://10.250.32.1:11311`（板端 dummy 网卡 ros0，不跟随 Wi-Fi），本机经到它的 /32 路由访问，下一跳按本机网段自动选板端 Wi-Fi 地址（Xiaomi `192.168.31.32` 优先，其次 chuangxiangG209 `192.168.8.32`，都不在则解析 `orangepi5max.local`），缺路由时用 sudo 补一条；自动识别本机 ROS IP；
可通过 `--ros-master URI --ros-ip IP` 覆盖。保留原入口全部参数。
默认容器名 `rviz_mid360s`，同名容器会被替换；并行使用需传不同 `--name`。

## 显示与面板保护

- Fixed Frame 为 `drone_odom`（10-21 前叫 `world`），TF 来自板端重力桥接，本机不重复发布。
- 使用 `--ipc=host` 修复本机 Qt5/XCB 的面板滚动残影；共享宿主机 IPC，限用于可信的本机显示镜像。
- 不自动最大化或还原主窗口；旧 `--no-maximize` 为兼容空操作，`--maximize-delay N` 接受但忽略。
- 默认保留 Displays/Views 和约 700px 左栏。启动布局稳定后仅在超过 12 MiB 保守面积预算时收窄分隔条。
- 手动检查/修复：在本目录运行 `./scripts/repair_layout.sh --report` 或 `./scripts/repair_layout.sh`。
- `OV13855 Preview` 图像面板订阅 `/ov13855_apriltag_vision/image_preview`，Transport Hint 为 `compressed`
  （960px 带标注预览，约 15Hz、1 MB/s，由 orange 工作空间视觉节点发布）。不要改成 raw 或订阅原分辨率话题，
  raw 约 55 MB/s 会占满 Wi-Fi。板端 `scripts/fusion.sh` 默认经 `scripts/launch/camera.launch` 拉起相机与视觉节点
  （`--no-camera` 关闭）；单独启动相机/视觉时须先 `source ~/mid360s_ws/scripts/ros_net.sh`，否则本机收不到图像。
  个人配置 `home/ros/.rviz/` 只在不存在时从模板复制，已有个人配置需自行 Add 该面板或删除后重新生成。
- 保留 GPU 渲染、历史/当前点云与路径配置。2026-09-26 在旧入口完成隔离容器的 IPC 对照、滚动/加宽复测，用户确认手动最大化正常；不是板端实时点云验收。

`local.env`、`home/`、`runtime/`、`logs/` 和 `.image.env` 为自动生成的本机数据，已忽略，
不迁入旧目录中的授权文件、用户缓存、日志或 bag。首次运行自动创建所需目录。
旧 `/home/zjh/docker/ros-noetic-rviz` 保留用于兼容已有会话，今后维护本仓库版本。
