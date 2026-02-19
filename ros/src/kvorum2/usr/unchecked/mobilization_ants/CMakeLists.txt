cmake_minimum_required(VERSION 2.8.3)
project(mobilization_ants)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msg_kvorum
)

catkin_package(
  INCLUDE_DIRS ../../lib/
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

