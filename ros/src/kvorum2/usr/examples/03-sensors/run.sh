#!/bin/bash

MODULE=demo
SDIR=`pwd`
CFGDIR=$SDIR/config

scriptdir=`rospack find kvorum2`/utils
$scriptdir/map2ctl.py "$CFGDIR/field.map" "$CFGDIR/map.ctl" "$CFGDIR/env.ctl" plan.p

#
#
# Launch Viz
konsole -p tabtitle="Viz" -p TerminalRows=15 -p TerminalColumns=50 -e $SDIR/$MODULE.v &
echo "Wait... "
sleep 2
echo "Done."

# Launch Mod
konsole -p tabtitle="Mod" -p TerminalRows=15 -p TerminalColumns=50 -e $SDIR/$MODULE.m &
echo "Wait... "
sleep 2
echo "Done."

#
#
#

wmctrl  -a "mc"
wmctrl  -a "Kvorum_V"

# Launch main script
$SDIR/$MODULE.py $CFGDIR/env.ctl $CFGDIR/map.ctl $CFGDIR/agents.ctl

#
#
#
rosnode kill `rosnode list |grep kvorum`
