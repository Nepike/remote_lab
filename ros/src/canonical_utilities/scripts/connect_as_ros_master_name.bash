#!/usr/bin/env bash
HOSTNAME=$1

export ROS_MASTER_URI=http://${HOSTNAME}:11311
export ROS_HOSTNAME=${HOSTNAME}
