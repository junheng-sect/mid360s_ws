# 本机 Docker RViz

本目录随 mid360s_ws 管理显示脚本、Dockerfile、RViz/launch 配置与面板测量工具。
用于 Ubuntu 桌面 X11 + NVIDIA Docker 环境，不在 Orange Pi 板端运行。

在仓库根目录调用（仓库可放在任意路径，用户名不必相同）：

```bash
./scripts/run_rviz.sh
./scripts/run_rviz.sh --detach
./scripts/run_rviz.sh --help
```

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

默认 ROS Master 为 `http://192.168.31.32:11311`，自动识别本机 ROS IP；
可通过 `--ros-master URI --ros-ip IP` 覆盖。保留原入口全部参数。
默认容器名 `rviz_mid360s`，同名容器会被替换；并行使用需传不同 `--name`。

## 显示与面板保护

- Fixed Frame 为 `world`，TF 来自板端重力桥接，本机不重复发布。
- 使用 `--ipc=host` 修复本机 Qt5/XCB 的面板滚动残影；共享宿主机 IPC，限用于可信的本机显示镜像。
- 不自动最大化或还原主窗口；旧 `--no-maximize` 为兼容空操作，`--maximize-delay N` 接受但忽略。
- 默认保留 Displays/Views 和约 700px 左栏。启动布局稳定后仅在超过 12 MiB 保守面积预算时收窄分隔条。
- 手动检查/修复：在本目录运行 `./scripts/repair_layout.sh --report` 或 `./scripts/repair_layout.sh`。
- 保留 GPU 渲染、历史/当前点云与路径配置。2026-09-26 在旧入口完成隔离容器的 IPC 对照、滚动/加宽复测，用户确认手动最大化正常；不是板端实时点云验收。

`local.env`、`home/`、`runtime/`、`logs/` 和 `.image.env` 为自动生成的本机数据，已忽略，
不迁入旧目录中的授权文件、用户缓存、日志或 bag。首次运行自动创建所需目录。
旧 `/home/zjh/docker/ros-noetic-rviz` 保留用于兼容已有会话，今后维护本仓库版本。
