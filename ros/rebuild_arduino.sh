#!/bin/bash

source ~/ros/devel/setup.bash

rm -r -f ~/ros/arduino/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py ~/ros/arduino/libraries

#rosrun rosserial_client make_libraries ~/ros/arduino/libraries
