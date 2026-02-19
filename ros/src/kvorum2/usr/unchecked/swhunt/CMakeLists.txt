cmake_minimum_required(VERSION 2.8.3)
project(swhunt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msg_kvorum
  msg_ans
  msg_rsaction
  cmake_modules
  tf
)

find_package(TinyXML REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp cmake_modules
  DEPENDS TinyXML
  INCLUDE_DIRS  ../../../lib ../../lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_executable(swhunt src/swhunt.cpp  ../../../lib/erlib.cpp ../../../lib/servlib.cpp ../../lib/emt/emt.cpp ../../lib/auto/auto.cpp ../../lib/tmurobot/tmurobot.cpp ../../../lib/exec_wrapper/exec_wrapper.cpp)

add_dependencies(swhunt ${catkin_EXPORTED_TARGETS})

target_link_libraries(swhunt
   ${catkin_LIBRARIES}
   curses
   ${TinyXML_LIBRARIES}
)

