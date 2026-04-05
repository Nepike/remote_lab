cmake_minimum_required(VERSION 3.0.2)
project(mobot_du)

find_package(catkin REQUIRED COMPONENTS
  xacro
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobot_du
#  CATKIN_DEPENDS xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

