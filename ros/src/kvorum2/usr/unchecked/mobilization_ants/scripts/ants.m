#!/bin/bash

SDIR=~/ros/src/kvorum/kvorum_m/scripts
CFGDIR=`rospack find mobilization_ants`/config

$SDIR/mmain.py $CFGDIR/env.ctl $CFGDIR/map.ctl $CFGDIR/agents.ctl
