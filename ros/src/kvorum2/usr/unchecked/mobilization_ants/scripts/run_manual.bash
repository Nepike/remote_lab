#!/bin/bash

PYTHONPATH=$PYTHONPATH:~/ros/src/kvorum/pylib

rosrun mobilization_ants manual_control.py _robot_id:=1
