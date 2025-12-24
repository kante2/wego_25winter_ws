cmake_minimum_required(VERSION 3.0.2)
project(wego_cfg)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/LaneDetect.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)
