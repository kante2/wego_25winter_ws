cmake_minimum_required(VERSION 3.0.2)
project(decision_wego)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ackermann_msgs
  geometry_msgs
  dynamic_reconfigure
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs ackermann_msgs geometry_msgs dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/lane_detect_decision_solo.py
  scripts/obstacle_avoid_decision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
