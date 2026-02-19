#!/usr/bin/env bash

echo Loading agents...


for i in `seq 1 20`; do
	echo "Loading agent #$i"
	roslaunch mobilization_ants yarp4.launch __ns:=robot$i > /dev/null &
done

echo Finished loading
