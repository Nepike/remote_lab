#!/bin/bash
SDIR=`rospack find yyctl`/scripts/aruco/

$SDIR/aruco_server.py --input /g1/group_mobot1/mobot1/camera1/image_raw  --cfg $SDIR/cfg/camera --topic superlocator --tmr 10
