#!/bin/bash

cfgfile=`rospack find gmodctl`/scripts/config/robot.cfg
`rospack find gmodctl`/scripts/local_comm_emulator/lce.py --cfg $cfgfile

