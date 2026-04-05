#!/bin/bash
cfgdir=`rospack find gmodctl`/scripts/config

./servoctl.py --cfg $cfgdir/robot.cfg --rid 1
