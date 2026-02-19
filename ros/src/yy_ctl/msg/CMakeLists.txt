cmake_minimum_required(VERSION 2.8.3)
project(msg_yy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
    FILES
    cmd.msg
    sens.msg
    arucopoint.msg
    arucopos.msg
    bt_data.msg
    servolocator.msg
    dtm.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
   std_msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

