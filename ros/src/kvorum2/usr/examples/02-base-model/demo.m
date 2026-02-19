#!/bin/bash

SDIR=`rospack find kvorum2`/kvorum_m
CFGDIR=./config

$SDIR/mmain.py $CFGDIR/env.ctl $CFGDIR/map.ctl $CFGDIR/agents.ctl

