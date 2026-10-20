cmake_minimum_required(VERSION 3.0.2)
project(drone_mapping)

find_package(catkin REQUIRED)

# Python 模块 src/drone_mapping/（devel 空间里导入）；入口 scripts/build_map 直接从包目录执行（rosrun 找得到），
# 工作空间不用 install 空间，所以不调 catkin_install_python（否则 rosrun 会找到两个同名入口）
catkin_python_setup()
catkin_package()
# 单元测试不挂到 catkin：python3 -m unittest discover -s src/drone_mapping/test（见 README）
