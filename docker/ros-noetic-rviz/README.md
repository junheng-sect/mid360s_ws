# 本机 Docker RViz

本目录随 mid360s_ws 管理显示脚本、Dockerfile、RViz/launch 配置与面板测量工具。
支持 Ubuntu 桌面 X11 + NVIDIA Docker，以及 Apple Silicon Mac 的 ARM64 Docker + 浏览器 RViz。
显示和辅助节点都在本机容器运行，不在 Orange Pi 板端增加进程。

## Apple Silicon Mac

安装 Docker Desktop 4.34 或更新版本，并启用 Settings → Resources → Network → Enable host networking。
若未创建系统级 CLI 链接，入口会自动使用 `/Applications/Docker.app/Contents/Resources/bin/docker`。
Ubuntu 原 `scripts/run_rviz.sh`、`scripts/build_image.sh`、`common.sh` 和 launch 文件保持原样；
Mac 使用独立入口，不通过原命令按系统分流。首次构建与日常使用（仓库根目录）：

```bash
./docker/ros-noetic-rviz/scripts/build_image_mac.sh
./scripts/run_rviz_mac.sh
./scripts/run_rviz_mac.sh --reloc
./scripts/run_rviz_mac.sh --check
./scripts/run_rviz_mac.sh --reloc --check
./scripts/run_rviz_mac.sh --stop
./scripts/run_rviz_mac.sh --dry-run
```

Mac 镜像 `ros-noetic-rviz:noetic-arm64-web` 使用固定摘要的 ARM64 `ros:noetic-ros-base`，
只安装显示依赖。默认使用 Mesa llvmpipe 软件渲染、1440×900 虚拟桌面和无 JPEG 的 noVNC，
自动打开 `http://localhost:6080/vnc.html`。浏览器画面可缩放、旋转和操作完整 RViz。
`--no-open` 可仅启动显示容器，不自动打开浏览器。
关闭 **RViz 窗口** 或 `--stop` 会退出桌面、辅助节点和容器；关闭浏览器标签页不会停止容器。
FAST-LIO 与重定位共用默认容器和端口，切换前先执行 `--stop`。
要并行打开 FAST-LIO 与重定位视图，使用不同的 `--name` 和 `--web-port`，例如：

```bash
./scripts/run_rviz_mac.sh --reloc --name rviz_reloc --web-port 6082
```

HTTP 和 VNC 仅监听本机回环地址；`--web-port` 范围为 1024–55535，另保留该端口加 10000 给 VNC。
个人配置首次初始化为适合虚拟桌面的窗口尺寸，后续保存不会被覆盖。
保持历史点云 **1000 秒**、原有话题与配色，不默认降采样；长时间运行的资源占用取决于累计点数。
EGO 独立显示在 Orange 仓库 `tools/ego_rviz` 中，默认使用浏览器端口 6081。

Mac 仍需与板端同网或有可达路由。入口自动识别宿主机源 IP，按需用 `sudo route` 添加到
`10.250.32.1` 的临时主机路由；`BOARD_IP` 可指定板端 Wi-Fi 下一跳，`--ros-master` 和 `--ros-ip` 可覆盖。
不会改变默认路由、板端配置或添加板端代理。
启动前会通过 **声明的宿主机 IP** 检查宿主机与容器双向 TCP；通过不代表外部板端回调已验收。
`--check` 实际接收所选视图的点云、里程计／重定位状态和 TF，列出 compressed 图像接收状态；
缺少必需数据时返回非零。Master 可达或话题已登记不能单独判定显示可用。

`--dry-run` 不调用 Docker、不创建目录、不添加路由、不生成 X 授权。`--shell`、`--cmd`、`--check`
不启动桌面。Mac 不使用 NVIDIA、宿主机 X11 或宿主机 ROS，也不扩展无人机状态桥或开机自启。

离线脚本回归：`python3 docker/ros-noetic-rviz/tests/test_launchers.py -v`。
脚本回归不代替接板端的点云／图像、回调及性能验收。
本次本机验证结果、性能观测和待验收项见 [Mac 验证记录](MAC_VALIDATION.md)。
重定位和 FAST-LIO 的 Mac RViz 节点分别按容器名命名，避免并行视图互相退出；
检查容器和检查节点也使用独立名称，可并行执行 `--check`。

本机快捷命令 `rviz-mac` 指向 `scripts/run_rviz_mac.sh`，可用 `rviz-mac --reloc` 切换视图。
Mac 只读取 `local.mac.env`，不读取 Ubuntu 的 `local.env`，构建记录写入 `.image.mac.env`。
按需创建配置（已有文件不覆盖）：

```bash
cp -n docker/ros-noetic-rviz/local.mac.env.example docker/ros-noetic-rviz/local.mac.env
```

配置优先级：命令行参数 > `local.mac.env` 的 RVIZ_* / EGO_* > 内置默认值；
ROS Master 未覆盖时也接受已有 `ROS_MASTER_URI`。Mac 使用 `Dockerfile.macos`、
`common_mac.sh`、`display_runtime_mac.sh` 和 `*_mac.launch`；原 Ubuntu 文件不加载这些文件。

## Ubuntu

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
./scripts/run_drone_status.sh --install   # 装成本机用户服务 drone-status，开机自启（每台电脑装一次，--uninstall 卸）
systemctl --user status drone-status      # 看状态；日志 journalctl --user -u drone-status -f
curl -s http://127.0.0.1:8091/status      # 汇总好的 JSON
```

不装服务时 `./scripts/run_drone_status.sh` 后台起容器 drone_status（`--fg` 前台看日志，`--stop` 停）；服务在跑时手动起停会被拒绝。
服务跑的是 `--fg`，退出 10 s 后重起，每次起都重选 ROS_IP、补路由；开机时本机还没连上网就一直重试。用户服务不登录也能开机起，
要求当前用户开了 linger（`loginctl show-user $USER -p Linger`，本机已开）。服务没有终端输 sudo 密码：要 sudo 补路由的网（手机热点等）
先手动跑一次 `run_rviz.sh` 或 `sudo ip route replace 10.250.32.1/32 via <板端地址>`。

`local_view/drone_status.py` 在容器里只读订阅板端常驻 MAVROS 的低频话题（state、battery、sys_status、extended_state、
estimator_status、送给飞控的视觉位姿），加上每 2 s 问一次板端 master（fusion 核心节点在不在、MAVROS 有没有重启），
汇总成一份 JSON，带“可以起飞 / 不可起飞”的判断。定位只在要位置的模式（POSCTL、OFFBOARD、AUTO.*）下算进总灯；
MANUAL、STABILIZED、ALTCTL、ACRO 不跑 fusion 也能起飞，定位一行只作参考（页面灰色“–”）。板端不加进程，只多一个订阅者；本节点不发布、不调服务、不写参数。
页面是 orange 仓库的 `tools/car_web`（顶栏“无人机”“机电量”，点“无人机”展开），默认取 `http://<打开页面的主机>:8091/status`，
手机经台式机 IP 打开也能用。镜像里没有 mavros_msgs，节点按发布者连接头里的消息定义现场解析，不用重建镜像。
板端没开时照样起，页面显示“连不上板端”；板端重启（master 换了）节点自己重来。本机换网后 ROS_IP 就失效了
（之后才起的 fusion 订阅不上）：ROS_IP 是自动识别的时，节点发现本机到板端的源地址连续 3 次（每 2 s 查一次）不是 ROS_IP 就退出，由服务重起
（换网途中这几秒页面显示“本机状态桥连不上”）；手动起的要重跑；`--ros-ip` 或 local.env 写死地址时不查。

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

`local.env`、`local.mac.env`、`home/`、`runtime/`、`logs/`、`.image.env` 和 `.image.mac.env`
为自动生成的本机数据，已忽略，
不迁入旧目录中的授权文件、用户缓存、日志或 bag。首次运行自动创建所需目录。
旧 `/home/zjh/docker/ros-noetic-rviz` 保留用于兼容已有会话，今后维护本仓库版本。
