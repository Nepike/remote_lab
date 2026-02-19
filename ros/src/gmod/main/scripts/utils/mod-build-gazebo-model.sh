#!/bin/bash
#
# Конвертация описания полигона Kvorum в модель Gazebo
# Аргументы: имя map-файла, размер ячейки в метрах
#

SDIR=`pwd`
CFGDIR="$SDIR/config"

if [ $# -lt 2 ]
then
  echo -e "\n*** Usage is: $0 map-file cellsize\n"
  exit 0
fi

#mapfile=$CFGDIR/room.map
# --cellsize 0.3
mapfile=$1
cellsize=$2

if [ -f $mapfile ]; then
  echo -e "Processing $mapfile"
else
  echo "*** file $mapfile not found"
  exit 1
fi

scriptdir=`rospack find gmodctl`/scripts/utils
modeldir=`rospack find polygone_mobot`/worlds

launchdirmobot=`rospack find mobot_du`/launch
launchdiraruco=`rospack find gmodctl`/launch
launchdirbridge=`rospack find gmodctl`/launch

# not used
launchdirrctl=`rospack find gmodctl`/launch


#
# Create ctl and plan files
#
echo ""
echo "STEP 1"
echo $mapfile '->' 
echo "  $CFGDIR/map.ctl"
echo "  $CFGDIR/env.ctl"
echo "  plan.p"

scriptdir2=`rospack find kvorum2`/utils
"$scriptdir2"/map2ctl.py --map "$mapfile" --ctl "$CFGDIR"/map.ctl --env "$CFGDIR"/env.ctl --plan plan.p

echo ""
echo "STEP 2"
echo $mapfile '->' 
echo "  $modeldir/world.inc"
echo "  $launchdirmobot/mobot.launch"
echo "  $launchdiraruco/aruco.launch"
echo "  $launchdirrctl/rctl.launch"
echo "  $launchdirbridge/bridge.launch"
echo "  $launchdirbridge/servolocator.launch"

# rctl.launch генерировать не будем
"$scriptdir"/map2world.py --map "$mapfile" --inc $modeldir/world.inc --launchmobot $launchdirmobot/mobot.launch --launcharuco $launchdiraruco/aruco.launch --launchbridge $launchdirbridge/mobot_du_kvorum_bridge.launch --launchsrv $launchdirbridge/servolocator.launch --cellsize $cellsize

if [ $? -ne 0 ]
then
  echo "***"
  echo "*** Error in map2world"
  echo "***"
  exit 1
fi

echo ""
echo "STEP 3"
echo $modeldir/model.world.xacro '->' $modeldir/model.world

xacro $modeldir/model.world.xacro -o $modeldir/model.world
if [ $? -ne 0 ]
then
  echo "***"
  echo "*** Error in xacro"
  echo "***"
  exit 1
fi

echo "Done"
