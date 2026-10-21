cmake_minimum_required(VERSION 3.0.2)
project(drone_relocalization)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
# Match drone_mapping's devel-space script convention (no duplicate rosrun entry).
install(PROGRAMS scripts/relocalize scripts/reloc_view DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md ACCEPTANCE.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} OPTIONAL)
