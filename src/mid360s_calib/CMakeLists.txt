cmake_minimum_required(VERSION 3.0.2)
project(mid360s_calib)

find_package(catkin REQUIRED)

catkin_package()

# 节点与它同目录 import 的两个纯逻辑模块一起安装（节点用 sys.path 找同目录模块）
catkin_install_python(
  PROGRAMS scripts/calib_bag_recorder.py scripts/rc_record_policy.py
           scripts/sequence_allocator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_calib_record.py)
endif()
