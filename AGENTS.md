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

- SSH 别名：`ssh orange`；完整登录方式：`ssh orangepi@192.168.31.32`。
- 板端用户：`orangepi`；SSH 和 sudo 密码均为 `[REDACTED]`，日常优先免密登录。
- 本机 SSH 配置：`~/.ssh/config` 的 `Host orange`；专用密钥：`~/.ssh/id_ed25519_orange`。
- 默认调试网络：Wi-Fi `Xiaomi`，接口 `wlan0`，固定地址 `192.168.31.32/24`，网关和 DNS 均为 `192.168.31.1`。
- NetworkManager 的 `Xiaomi` 连接开机自动连接，优先级 `100`，重试次数 `0`（持续重试）；SSH 服务开机启动。
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
