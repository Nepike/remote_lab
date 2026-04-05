#!/bin/bash
#
# Активизировать rosserial для всех нормальных портов
#
list=`ls /dev/ttyUSB? /dev/ttyACM?`

let n=0
for port in $list
do
  let n++
  echo "rid=$n, port=$port"
  konsole -p tabtitle="node$n: $port" -p TerminalRows=15 -p TerminalColumns=50 -e "roslaunch yyctl rosserial.launch port:=$port rid:=$n" &
  sleep 2
done

