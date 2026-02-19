#!/bin/bash

SDIR=~/ros/src/kvorum/kvorum_v/scripts
CFGDIR=`rospack find mobilization_ants`/config

$SDIR/vmain.py $CFGDIR/env.ctl $CFGDIR/map.ctl

