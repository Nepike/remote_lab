#!/bin/bash

# Читатель
# Прием данных и запись в файл

myaddr=2

resfile=`pwd`/result$myaddr.txt

#
#topicname=/xbeeds_cmd$myaddr
#topictype=msg_yy/dtm
#
#
# 3 - просто прочитать
# 4 - прочитать и записать в файл (data - имя файла c данными)
# cmd=3
#rostopic pub -1 $topicname $topictype $cmd 0 0 "$data"
#

`rospack find yyctl`/scripts/communication/zigbee/xbee_ds.py --addr $myaddr --output $resfile

