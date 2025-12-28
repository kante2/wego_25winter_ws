cmake_minimum_required(VERSION 3.0.2)
project(perception_wego)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  dynamic_reconfigure
  wego_cfg
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge dynamic_reconfigure wego_cfg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/lane_detect_perception.py
  scripts/obstacle_avoid_perception.py
  scripts/aruco_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
