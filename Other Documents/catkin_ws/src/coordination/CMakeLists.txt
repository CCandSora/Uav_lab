cmake_minimum_required(VERSION 3.0.2)
project(coordination)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/coordinator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

