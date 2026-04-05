#!/bin/bash

if [ $# -lt 1 ];
then
        echo "Usage is: $0 text"
        exit
fi

#echo $1 | spd-say -o rhvoice -l ru  -e -t female1

echo $1 $2 $3 $4 $5 $6 $7 $8 $9 | RHVoice-client -s irina+CLB | aplay
