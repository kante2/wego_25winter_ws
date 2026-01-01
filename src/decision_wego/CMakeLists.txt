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
  scripts/main_node.py
  scripts/mission_lane.py
  scripts/mission_lane_ver2.py
  scripts/mission_obstacle.py
  scripts/mission_parking.py
  scripts/mission_crosswalk.py
  scripts/mission_traffic_light.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
