#!/bin/bash

MODULE=smp_demo
SDIR=`pwd`
CFGDIR=$SDIR/config


#
#
# Launch Viz
konsole -p tabtitle="Viz" -p TerminalRows=15 -p TerminalColumns=50 -e $SDIR/demo.v &
echo "Wait... "
sleep 2
echo "Done."

# Launch Mod
konsole -p tabtitle="Mod" -p TerminalRows=15 -p TerminalColumns=50 -e $SDIR/demo.m &
echo "Wait... "
sleep 2
echo "Done."

#
#
#

wmctrl  -a "mc"
wmctrl  -a "Kvorum_V"

#
# Launch main script
#
$SDIR/$MODULE.py $CFGDIR/env.ctl $CFGDIR/map.ctl $CFGDIR/agents.ctl

#
#
#
rosnode kill `rosnode list |grep kvorum`
