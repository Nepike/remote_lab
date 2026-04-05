#!/bin/bash

# Передача файла

myaddr=1
recepient=2
filename=`pwd`/data1.txt

#topicname=/xbeeds_cmd1
#topictype=msg_yy/dtm
# Команды передатчику
#CMD_SEND_DATA = 1 # arg1 - адрес получателя. В поле data - передаваемые данные
#CMD_SEND_FILE = 2 # arg1 - адрес получателя. В поле data - имя файла
#cmd=2
#rostopic pub -1 $topicname $topictype $cmd $recepient 0 "$filename"


`rospack find yyctl`/scripts/communication/zigbee/xbee_ds.py --addr $myaddr --recepient $recepient --input $filename
