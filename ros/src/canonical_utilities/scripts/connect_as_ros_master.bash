#!/usr/bin/env bash
IP=$1

export ROS_MASTER_URI=http://${IP}:11311
export ROS_IP=${IP}
