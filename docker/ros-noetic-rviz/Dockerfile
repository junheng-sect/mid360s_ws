# RViz 显示容器：复用本机已有的 osrf/ros:noetic-desktop-full，按摘要锁定基础镜像。
# 本容器只负责“看图”，不装 Livox 驱动、不编译 FAST-LIO2，算法与驱动都在板端运行。
FROM osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797

# Ubuntu 20.04 官方源在本机较慢，换成与宿主机一致的 USTC 镜像；ROS 源保持原样。
RUN set -eux; \
    sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu|http://mirrors.ustc.edu.cn/ubuntu|g' \
      -e 's|http://security.ubuntu.com/ubuntu|http://mirrors.ustc.edu.cn/ubuntu|g' \
      /etc/apt/sources.list; \
    apt-get update -o Acquire::Retries=3; \
    apt-get install -y --no-install-recommends \
      mesa-utils \
      x11-utils \
      xauth \
      iproute2 \
      python3-websocket \
      net-tools \
      procps; \
    rm -rf /var/lib/apt/lists/*

# 容器以宿主机 UID/GID 运行（见 scripts/run_rviz.sh），HOME 由宿主机目录挂载到 /home/ros。
# 这里只保证目录存在且可写，避免 RViz 因缺少 HOME 而报配置写入失败。
RUN mkdir -p /home/ros && chmod 777 /home/ros
ENV HOME=/home/ros

# 保留旧 Qt 环境兼容变量；它不能保证禁用 Qt5/XCB 的 MIT-SHM。
# 本机 RViz 显示/回放入口使用 --ipc=host 修复 Qt5 停靠面板重绘，见 README。
ENV QT_X11_NO_MITSHM=1

WORKDIR /home/ros
