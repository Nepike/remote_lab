#!/usr/bin/env sh

if [ -z "$1" ]
  then
    BAG_FILE=data_0
  else
    BAG_FILE=$1
fi

BAG_PATH=~/bags

rosparam set /use_sim_time True

rosrun rosbag play ${BAG_PATH}/${BAG_FILE}.bag -d 2 &
