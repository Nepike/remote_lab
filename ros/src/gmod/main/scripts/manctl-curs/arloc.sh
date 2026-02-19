#!/bin/bash
SDIR=`rospack find yyctl`/scripts/aruco/

# robot`s id
RID=1

$SDIR/aruco_server.py --input /group_mobot$RID/mobot$RID/camera1/image_raw --cfg $SDIR/cfg/camera --topic superlocator --tmr 10
