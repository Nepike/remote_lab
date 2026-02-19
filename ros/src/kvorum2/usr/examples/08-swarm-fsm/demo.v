#!/bin/bash

SDIR=`rospack find kvorum2`/kvorum_v
CFGDIR=./config

$SDIR/vmain.py $CFGDIR/env.ctl $CFGDIR/map.ctl

