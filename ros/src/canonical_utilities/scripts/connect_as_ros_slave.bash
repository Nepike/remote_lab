#!/usr/bin/env bash
MASTER_IP=$1
MY_IP=$2

export ROS_MASTER_URI=http://${MASTER_IP}:11311
export ROS_IP=${MY_IP}
