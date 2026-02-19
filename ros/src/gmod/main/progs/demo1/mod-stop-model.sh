#!/bin/bash
#
# Останов сервисов и процессов
#

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

exit
