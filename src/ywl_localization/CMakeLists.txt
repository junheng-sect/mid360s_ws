cmake_minimum_required(VERSION 3.10)
project(ywl_localization)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy geometry_msgs nav_msgs sensor_msgs diagnostic_msgs pcl_conversions
  livox_ros_driver2)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenSSL REQUIRED)
catkin_package()
add_executable(ywl_localization_node src/localization.cpp src/preprocess.cpp include/ikd-Tree/ikd_Tree.cpp)
target_include_directories(ywl_localization_node PRIVATE include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(ywl_localization_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  Eigen3::Eigen OpenMP::OpenMP_CXX OpenSSL::Crypto)
add_dependencies(ywl_localization_node ${catkin_EXPORTED_TARGETS})
catkin_install_python(PROGRAMS scripts/ywl_init_adapter scripts/ywl_replay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ywl_localization_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
