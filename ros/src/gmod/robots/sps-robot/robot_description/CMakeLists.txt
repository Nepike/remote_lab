cmake_minimum_required(VERSION 3.0.4)
project(test_robot_description)

find_package(catkin REQUIRED roscpp)
find_package(gazebo REQUIRED)

#find_package(catkin REQUIRED hector_sensors_description)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(contact_handler_node src/contact_handler.cpp)
target_link_libraries(contact_handler_node ${GAZEBO_LIBRARIES} pthread ${catkin_LIBRARIES})

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )
