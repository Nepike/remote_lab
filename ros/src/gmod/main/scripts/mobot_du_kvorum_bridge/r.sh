#!/bin/bash
cfgdir=`rospack find gmodctl`/scripts/config

./mobot_du_kvorum_bridge.py --cfg $cfgdir/robot.cfg --rid 1
