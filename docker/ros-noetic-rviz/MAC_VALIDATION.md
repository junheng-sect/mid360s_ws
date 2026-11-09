# Mac Docker RViz 本机验证记录

日期：2026-11-09。MacBook Air / Apple Silicon，macOS 27.2，Docker Desktop 4.98.0。
当前 Mac 在校园网 `10.180.19.197`，与板端所在网络不同；以下数据验证均为本机隔离测试。

## 当前入口：Ubuntu 原命令与 Mac 独立命令

按用户要求，Ubuntu 原入口已恢复，不再按系统自动分流。

| 平台 | FAST-LIO／重定位 | EGO | 构建与配置 |
|---|---|---|---|
| Ubuntu | 原 `scripts/run_rviz.sh`，重定位加 `--reloc` | 原 `ego-rviz` / `scripts/run.sh` | 原 Dockerfile、build_image.sh、common.sh、local.env |
| Mac | 新 `scripts/run_rviz_mac.sh` / `rviz-mac`，重定位加 `--reloc` | 新 `scripts/run_mac.sh` / `ego-rviz-mac` | Dockerfile.macos、build_image_mac.sh、common_mac.sh、local.mac.env |

Mac 使用独立 `*_mac.launch` 和 `display_runtime_mac.sh`，构建记录为 `.image.mac.env`。
两仓库共 12 个 Ubuntu 原入口、网络、构建、检查、Dockerfile、配置示例和 launch 文件
已逐字节与修改前的仓库 HEAD 比较，一致。原 EGO Dockerfile 和 `--check` 的行为也已恢复。
Mac 不读取 Ubuntu 的 local.env / IMAGE_TAG，原脚本不导入 Mac 文件。
本机此前为 Mac 创建的 `ego-rviz` 链接已移除，新链接为 `rviz-mac` 与 `ego-rviz-mac`。
拆分后通过新构建命令重建两 ARM64 镜像，使用新快捷命令重新启动三个显示容器，
三种 `--check` 并行接收合成数据并返回 0；新 `--stop` 命令释放全部显示端口，测试容器已清理。

## 已验证

- Docker host networking 已启用；使用本机实际地址 `10.180.19.197` 的宿主机 ↔ 容器双向 TCP 探测通过。
- 两个镜像均为 `arm64`。基础镜像锁定为
  `ros:noetic-ros-base@sha256:c84e573d2f2beb679ea5b03015eb2876e9962657c23d0073e4ae9d7ba41c4700`。
  EGO 仅在容器构建官方显示组件；`glxinfo -B` 确认 Mesa llvmpipe，OpenGL 4.5，非硬件加速。
- FAST-LIO、重定位、EGO 可同时运行，测试端口分别为 6080、6082、6081。
  虚拟 X 显示按 web 端口分开，Mac RViz 节点按容器名分开；并行 `--check` 节点用 UUID 防止跨容器撞名。
- 13 项脚本回归通过：独立 Mac 运行／构建命令、Ubuntu 原文件内容保留、拒绝在其他系统执行 Mac 入口、
  两平台配置隔离、参数覆盖、重定位配置、带空格路径、EGO 符号链接、选图／里程计、
  无桌面检查、无副作用 dry-run、个人布局保留。
  `/bin/bash` 3.2 语法检查、Python 语法解析及两仓库 `git diff --check` 通过。
- 个人布局首次初始化为 1380×820，桌面为 1440×900；历史点云仍为 1000 秒，
  配色／话题保持原模板，图像预览使用 compressed。后续保存不重写公共模板。
- FAST-LIO 浏览器画面已验证点云显示、旋转、缩放及保存个人布局。
  重定位和 EGO 已验证容器、ROS 节点、RFB／HTTP 服务就绪；这两套视图的浏览器交互仍待验收。
- 重定位失效：显示 `FAILED - not localized, drone not shown`，无人机标签发布 DELETE，
  缺少 `map <- camera_init` 时 `--reloc --check` 返回 1。
  测试夹具提供有效状态与变换后，标签恢复 ADD，状态变为 LOCALIZED，检查返回 0。
- EGO 使用 81 点合成 PCD：`saved_map` 完整收到 81 点、frame 为 `drone_map`，文件哈希保持不变、只读挂载。
  无人机模型输出为 `package://odom_visualization/meshes/hummingbird.mesh`。
  容器仅提供显示原点 `drone_map -> ego_swarm_map_origin`，不生成到实时 `drone_odom` 的兜底变换。
  缺少该真实变换时 EGO 地图检查返回 1；测试夹具提供变换后返回 0。
- 本机三种 `--check` 同时实际接收点云、TF、里程计／重定位状态和压缩图像，返回 0。
  `ADVERTISED_XMLRPC_OK` 在本机隔离网络通过；这不能替代板端回调验收。
- 在隔离 Master 上用 `rosnode kill` 模拟 RViz 退出：FAST-LIO 与 EGO 容器退出码 0，辅助服务随之退出。
  重定位 `--stop` 退出码 143（正常 SIGTERM），所有 HTTP／VNC 端口释放。
  测试显示容器、测试 Master 和夹具均已删除，构建镜像保留；没有清理其他 Docker 数据。

## 数据与性能范围

测试 Master 固定为 `http://127.0.0.1:21111`，ROS_IP 为 `127.0.0.1`。
`tests/offline_fixture.py` 拒绝其他 Master，画面与状态明确标记 `OFFLINE DISPLAY TEST`；
正常显示入口不启动测试夹具。未运行板端硬件、定位、规划或控制程序。

| 场景 | 帧率 | Docker CPU | 容器内存 |
|---|---|---|---|
| FAST-LIO，6561 点云以 5Hz 反复发布，保留 1000 秒历史，短时累积 | 观察约 0–4 FPS | 曾约 565% | 曾约 932 MiB |
| 单次 6561 点云 + 5Hz 合成里程计／图像，三视图并行，FAST-LIO | 未重新测量 | 143% | 210 MiB |
| 同上，重定位视图 | 未测量 | 100% | 263 MiB |
| 同上，EGO + 81 点合成 PCD | 未测量 | 85% | 206 MiB |

CPU 100% 表示约一个逻辑核心，数值是短时观测。测试没有跑满 1000 秒，
没有本机原始 4175519 点地图，不能据此承诺实机帧率或大地图流畅度。
5Hz 合成图像与里程计的订阅回调时间戳差约 1ms；单次 latched 点云约 0.15s。
这些数值不包含板端网络和浏览器画面传输延迟。软件渲染下持续累计点云明显增加 CPU／内存，
本次按要求保持原配置，不自动降采样。

## 接板端后继续验收

1. Mac 接到板端 Wi-Fi 或可达网络，必要时指定 `BOARD_IP`，检查到 `10.250.32.1/32` 的主机路由和自动识别的 ROS_IP。
2. 执行正常入口及 `--check`，记录实际点云、TF、里程计、重定位状态和压缩图像接收量、频率与时间戳差。
   Master 连通、话题登记或本机 TCP 探测不能单独判为通过。
3. 在板端现有 ROS 环境用 `rosnode ping /mac_rviz_rviz_mid360s` 检查 Mac RViz 的回调可达，
   并确认现有发布者的真实 TCPROS 连接和后续数据更新；命令行检查不新增常驻进程。
4. 在浏览器验收重定位／EGO 的旋转、缩放、图像、模型和个人布局保存，以及实际关窗退出。
5. 从本机原始 `map_world.pcd` 加载大地图，核对完整点数；用真实重定位变换验证无人机叠加与失效提示。
6. 在大地图与持续真实点云条件下记录 FPS、CPU、内存和从采集到浏览器显示的延迟。
7. 在 Ubuntu NVIDIA/X11 桌面实际启动回归。原入口及其依赖已恢复并逐字节核对，尚未在 Ubuntu 图形桌面运行。

后续浏览器工具访问本机页面未获授权，本次没有重试或改用其他方式获取画面。
若外部 ROS 双向通信失败，记录失败的路由／Master／XMLRPC／TCPROS 环节后继续讨论，
不增加板端桥接或更换方案。没有提交或推送代码，本机 AGENTS.md／个人布局／地图缓存仍不入库。
Ubuntu 本机说明中的共同环境事实在该电脑可达时再手动同步。
