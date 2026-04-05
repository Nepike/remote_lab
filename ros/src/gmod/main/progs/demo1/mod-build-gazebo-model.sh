#!/bin/bash
#
# Конвертация описания полигона Kvorum в модель Gazebo
#

SDIR=`pwd`
CFGDIR="$SDIR/config"

mapfile=$CFGDIR/room.map

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

scriptdir2=`rospack find kvorum`/utils
"$scriptdir2"/map2ctl.py "$mapfile" "$CFGDIR"/map.ctl "$CFGDIR"/env.ctl plan.p

echo ""
echo "STEP 2"
echo $mapfile '->' 
echo "  $modeldir/world.inc"
echo "  $launchdirmobot/mobot.launch"
echo "  $launchdiraruco/aruco.launch"
echo "  $launchdirrctl/rctl.launch"
echo "  $launchdirbridge/bridge.launch"
echo "  $launchdirbridge/servolocator.launch"

"$scriptdir"/map2world.py --map "$mapfile" --inc $modeldir/world.inc --launchmobot $launchdirmobot/mobot.launch --launcharuco $launchdiraruco/aruco.launch --launchbridge $launchdirbridge/bridge.launch --launchsrv $launchdirbridge/servolocator.launch --ratio 10

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
