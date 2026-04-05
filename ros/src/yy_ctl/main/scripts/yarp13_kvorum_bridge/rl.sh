#!/bin/bash

# Run hardware
roslaunch yyctl yarp13_kvorum_bridge.launch rotvang:=0.05 rotvlin:=0.07 cspeed:=0.25 cellsize:=0.3
