#!/bin/bash

PYTHONPATH=$PYTHONPATH:~/ros/src/kvorum/pylib

rosrun mobilization_ants simple_ants.py _robots_num:=1
