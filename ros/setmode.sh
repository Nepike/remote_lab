#!/bin/bash

if [ -f ./src/CMakeLists.txt ]; then
  rm ./src/CMakeLists.txt
fi

ln -s ~/ros/arduino/libraries/rcproto ~/ros/src/lib/rcproto

if [ $? -eq 0 ]
then
  echo "-- Successfully created link rcproto"
else
  echo -e "\n**** ERROR: Could not create link\n"
  echo -e "Try deleting the 'rcproto' directory in 'src/lib'"
fi

#chmod a+x buildall clr

#chmod a+x etc/lpp/start
#chmod a+x etc/mpp/start
#chmod a+x etc/drr/start
#chmod a+x etc/kvorum2/start
#chmod -R a+x scripts

#
#

find src/kvorum? -name '*.py' -exec chmod a+x {} \;
find src/kvorum? -name '*.m'  -exec chmod a+x {} \;
find src/kvorum? -name '*.v'  -exec chmod a+x {} \;
find src/kvorum? -name 'run*' -exec chmod a+x {} \;


find . -name '*.py'   -exec chmod a+x {} \;
find . -name '*.bash' -exec chmod a+x {} \;
find . -name '*.sh' -exec chmod a+x {} \;

echo -e "\nDone\n"
exit

chmod +x `rospack find reflex_move_controller`/scripts/reflex_move_cmd.py
chmod +x `rospack find drrobot_odom`/scripts/encoders_odom.py
chmod +x `rospack find crutches`/scripts/turn_server.py
chmod +x `rospack find crutches`/scripts/fwd_step_server.py
chmod +x `rospack find object_observer`/scripts/object_observer_node.py
chmod +x `rospack find state_machine_processor`/scripts/smp.py
chmod +x `rospack find command_server`/scripts/server.py
chmod +x `rospack find command_server`/scripts/telemetry_server.py
chmod +x `rospack find multibot_config`/scripts/connect_camera.bash

echo -e "\nDone\n"
