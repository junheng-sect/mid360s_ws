cmake_minimum_required(VERSION 3.0.2)
project(fastlio_gravity_bridge)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf2 tf2_ros
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(gravity_bridge_node
  src/gravity_bridge_node.cpp
  src/gravity_bridge.cpp)
target_link_libraries(gravity_bridge_node ${catkin_LIBRARIES})

install(TARGETS gravity_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_alignment_math test/test_alignment_math.cpp)
  if(TARGET test_alignment_math)
    target_link_libraries(test_alignment_math ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_covariance_transform test/test_covariance_transform.cpp)
  if(TARGET test_covariance_transform)
    target_link_libraries(test_covariance_transform ${catkin_LIBRARIES})
  endif()

  add_rostest_gtest(test_bridge_messages
    test/bridge_messages.test
    test/test_bridge_messages.cpp
    src/gravity_bridge.cpp)
  if(TARGET test_bridge_messages)
    target_link_libraries(test_bridge_messages ${catkin_LIBRARIES})
  endif()
endif()
