#!/bin/bash
#
# Конвертация описания полигона Kvorum в модель Gazebo
#

#mapfile=map1.map
mapfile=ori6.map

launchdir=`rospack find mobot_du`/launch
scriptdir=`rospack find gmodctl`/scripts/utils
modeldir=`rospack find polygone_mobot`/worlds

echo "STEP 1"

echo $mapfile '->' 
echo "  world.inc"
echo "  $launchdir/mobot.launch"


$scriptdir/map2world.py --map $mapfile --inc $modeldir/world.inc --launch $launchdir/mobot.launch --ratio 10

if [ $? -ne 0 ]
then
  echo "***"
  echo "*** Error in map2world"
  echo "***"
  exit 1
fi

echo ""
echo "STEP 2"
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
