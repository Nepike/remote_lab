cmake_minimum_required(VERSION 2.8.3)
project(temper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msg_kvorum
)

catkin_package(
  INCLUDE_DIRS ../../../lib/ ../../lib/emt
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(temper src/temper.cpp ../../../lib/erlib.cpp ../../../lib/servlib.cpp ../../lib/emt/emt.cpp ../../../lib/exec_wrapper/exec_wrapper.cpp)

add_dependencies(temper ${catkin_EXPORTED_TARGETS})

target_link_libraries(temper
   ${catkin_LIBRARIES}
   curses
)
