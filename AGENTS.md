# Orange Pi 工作说明

## 项目环境

- 板端：Orange Pi 5 Max，RK3588，Ubuntu 20.04.6 LTS，主机名 `orangepi5max`。
- 板端实际工作空间：`/home/orangepi/mid360s_ws`。
- 本机挂载后的工作空间：`/home/zjh/mnt/orange_home/mid360s_ws`，与板端上述目录是同一份文件。
- 本机独立仓库：`/home/zjh/mid360s_ws`，不是板端挂载目录。
- 本机系统：Ubuntu 24.04.4 LTS；本机 sudo 密码：`[REDACTED]`。
- 板端已安装并验证 ROS Noetic（`desktop-full`）和 `catkin-tools 0.9.4`，`catkin build` 可用，无需重复安装。
- ROS 安装目录：`/opt/ros/noetic`；catkin 命令：`/usr/bin/catkin`，使用板端 Python 3 环境。

## 两端分工与同步（必读）

项目通过 GitHub 仓库 `junheng-sect/mid360s_ws` 同步。区分板端工作空间、本机挂载入口和本机独立仓库：

- **Orange Pi 板端**（`/home/orangepi/mid360s_ws`）：唯一代码开发与实机运行端。代码修改针对这份工作空间，commit、push、catkin 构建和 ROS 节点运行通过 SSH 在板端执行。
- **本机挂载入口**（`/home/zjh/mnt/orange_home/mid360s_ws`）：通过 SSHFS 访问板端文件，编辑会直接修改板端，无需 Git 同步。挂载前先确认 `orange-mount` 成功，避免将文件误写到未挂载的本机目录。
- **本机独立仓库**（`/home/zjh/mid360s_ws`）：默认只读查看与分析，不构建、不运行 ROS 节点、不修改代码、不 commit、不 push；用户明确要求的文档维护按该次指令执行。
- 独立仓库的更新方向：板端 push 后，本机在对应分支执行 `git pull`；不要将独立仓库与挂载目录混为一谈。
- 实机验证以板端实际文件和运行结果为准，不以本机独立仓库的状态替代。

## SSH、网络与挂载

- SSH 别名：`ssh orange`（本机按所在网段自动选地址，见下）；完整登录方式：Xiaomi 下 `ssh orangepi@192.168.31.32`，chuangxiangG209 下 `ssh orangepi@192.168.8.32`，其他网 `ssh orangepi@orangepi5max.local`。
- 板端用户：`orangepi`；SSH 和 sudo 密码均为 `[REDACTED]`，日常优先免密登录。
- 本机 SSH 配置：`~/.ssh/config` 的 `Host orange`；专用密钥：`~/.ssh/id_ed25519_orange`。
- 板端 Wi-Fi（接口 `wlan0`，NetworkManager 连接均开机自动连接、重试次数 `0` 持续重试）按优先级连可见的网：`Xiaomi`（`100`，固定 `192.168.31.32/24`，网关和 DNS `192.168.31.1`）> `chuangxiangG209_5G`（`90`）> `chuangxiangG209`（`80`，2.4G 备用；两者固定 `192.168.8.32/24`，网关和 DNS `192.168.8.1`）> 手机热点 `zjhvivo`（`10`，DHCP）。已连着低优先级网时不会自动跳到后出现的高优先级网，要重启或 `nmcli con up Xiaomi`；SSH 服务开机启动。
- 板端 ROS 核心不跟随 Wi-Fi：dummy 网卡 `ros0`（NetworkManager 连接 `ros-core`，开机自动建立）上的板内固定地址 `10.250.32.1/32`，`~/mid360s_ws/scripts/ros_net.sh` 设 `ROS_IP=10.250.32.1`、`ROS_MASTER_URI=http://127.0.0.1:11311`，手动起节点前同样先 source 它；Wi-Fi 断开、换网、地址被收回都不影响板端节点之间的连接（2026-10-22 实测断网 40 s 无中断）。
- 雷达网口 `livox-mid360s` 固定 `192.168.1.5/24`，另钉 `192.168.1.194/32` 路由；Wi-Fi 分到 `192.168.1.x` 时 `/etc/NetworkManager/dispatcher.d/30-lidar-subnet-guard` 直接断开该 Wi-Fi 连接。
- 板端时钟：`fusion.sh`、`start_mid360s_fastlio.sh`（手持入口经它起 FAST-LIO）和实机 `build_map` 运行核心期间停 chronyd、收尾收拢残留进程后再启动（`mid360s_ws/scripts/clock_guard.sh`），运行中系统时间不跳；冻结期间起的进程带环境变量 `MID360S_RUN` / `MID360S_CLOCK_FROZEN=1`，roslaunch 被强杀留下的孤儿节点靠它找回。依赖 `/etc/sudoers.d/mid360s-chrony`（只放行免密 `systemctl stop/start chrony.service`）、chrony.service 的 drop-in `mid360s-core-guard.conf`（冻结锁 `/run/mid360s/clock_freeze.lock` 被占用或还有带 `MID360S_CLOCK_FROZEN=1` 的进程时拒绝启动）和 `/etc/tmpfiles.d/mid360s.conf`（开机建 `/run/mid360s`）。
- 板端常驻服务：开机起 systemd `mid360s-roscore`（常驻 master，板内地址同上）和 `mid360s-mavros`（飞控 `/dev/ttyS3:921600`，另开只绑 `127.0.0.1` 的本地口 udp `14555`，quiet mode），源文件和安装脚本在 `mid360s_ws/scripts/resident/`；`fusion.sh`、`start_handheld_test.sh`、实机 `build_map` 只用不起停，fusion 没跑时也能读飞控状态。飞控串口一直被 MAVROS 占着：直连串口（pymavlink 等）前先 `sudo systemctl stop mid360s-mavros`，用完 `start`；飞控校时 `fc_set_time.py` 默认走本地口。
- 板端 `avahi-daemon` 只在 `wlan0` 上广播，局域网内可用 `orangepi5max.local` 找到板端（手机热点等 DHCP 网络靠它）。
- 本机 `~/.ssh/config` 的 `orange`：本机有 `192.168.31.x` 地址用 `192.168.31.32`，否则有 `192.168.8.x` 用 `192.168.8.32`，都没有用 `orangepi5max.local`；`HostKeyAlias 192.168.31.32` 让各地址共用一份主机指纹。本机 RViz（`~/mid360s_ws/scripts/run_rviz.sh`，实现在 `docker/ros-noetic-rviz`）连 `http://10.250.32.1:11311`，经到 `10.250.32.1/32` 的路由访问，下一跳按同一规则选板端 Wi-Fi 地址（`BOARD_IP` 可覆盖）；本机 Xiaomi / chuangxiangG209 的 Wi-Fi 连接已带这条静态路由，其他网由 `run_rviz.sh` 用 sudo 补。
- 板端连不上任何已知网时：本机 `nmcli con up orange-rescue-ap` 在 `wlp8s0` 上开同名同密码的 `Xiaomi` 热点（本机 `192.168.31.1/24`，网线 `eno1` 保持上网），板端会以 `192.168.31.32` 连上来，进去改配置，用完 `nmcli con down orange-rescue-ap`。
- `orange-mount`：将板端 `/home/orangepi` 挂载到本机 `/home/zjh/mnt/orange_home`，默认可读写。
- `orange-umont`：正常卸载上述目录，命令名保留 `umont` 拼写；目录占用时先退出目录、关闭文件后重试。
- 两个命令位于本机 `~/.local/bin`，由普通用户运行，不使用 sudo；挂载手动触发，不自动随本机开机挂载。

## 协作与日志

- `AGENTS.md` 只保存长期有效的环境参数、连接凭据和协作规范，不记录每轮开发流水或操作命令清单。
- 日志统一写入本机 Obsidian 笔记目录：`/home/zjh/obsidian/note/<月.日>/LOG.md`。
- 日期目录必须使用不补零格式，例如 `6.12`，不要写成 `06.12`。
- 若当天目录或 `LOG.md` 不存在，先创建再记录。
- 每轮对话如果修改了文件，写入“修改记录”；如果只是排查或回答问题，写入“问题记录”。
- 不再默认向工作空间根目录追加 `LOG.md` 或 `PI_LOG.md`。

## 开发约定

- 优先按 Orange Pi 板端的实际系统、依赖和设备权限调试，不套用其他板卡的环境与服务配置。
- 板端 `~/.bashrc` 已自动加载 `/opt/ros/noetic/setup.bash`；非交互 SSH 命令或脚本中仍需显式 source 该文件，再执行构建或 ROS 命令。
- 工作空间构建完成后，使用其包前再 source `/home/orangepi/mid360s_ws/devel/setup.bash`；首次构建前该文件不存在属于正常情况，不代表 ROS 未安装。
- catkin 工作空间统一在板端使用 `catkin build`，不使用 `catkin_make`，不在本机挂载目录中构建或运行 ROS 节点。
- 涉及 launch 或 ROS 节点启动时，优先给出可直接复制执行的完整命令。
- 普通调试使用 `orangepi` 用户；相机、串口等设备先检查实际权限，仅对确需管理员权限的操作使用 sudo。
