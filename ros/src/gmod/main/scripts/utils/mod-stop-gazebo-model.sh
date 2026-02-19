#!/bin/bash
#
# Останов сервисов и процессов
#

USE_TMUX=1
SESSION="gazebo-model"


l=`rosnode list | grep /aruco_server_node`
if [ $? -eq 0 ]
then
  echo "kill $l"
  rosnode kill $l
fi

l=`rosnode list | grep /gmodctl_node`
if [ $? -eq 0 ]
then
  echo "kill $l"
  rosnode kill $l
fi

l=`rosnode list | grep /mobot_du_kvorum_bridge_node`
if [ $? -eq 0 ]
then
  echo "kill $l"
  rosnode kill $l
fi


l=`rosnode list | grep /servoctl_node`
if [ $? -eq 0 ]
then
  echo "kill $l"
  rosnode kill $l
fi

#rosnode kill /gazebo_gui
#rosnode kill /gazebo

#wmctrl -c gazebo_mobot_du
#wmctrl -c Gazebo


if [ $USE_TMUX -eq 1 ];
then
    tmux kill-session -t $SESSION
fi

