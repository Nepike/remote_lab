#!/bin/bash
#
# Запуск Gazebo-модели
# -- Собственно модель
# -- ArUco-сервер
# -- Серволокатор
# -- Монитор (ручное управление)
#

mwait()
{
  echo "Wait... "
  sleep 5
  echo "Done."
}

konsole -p tabtitle="gazebo_mobot_du" -p TerminalRows=15 -p TerminalColumns=50 -e "roslaunch mobot_du mobot.launch" &
mwait
mwait

konsole -p tabtitle="aruco" -p TerminalRows=15 -p TerminalColumns=50 -e "roslaunch gmodctl aruco.launch" &
mwait

konsole -p tabtitle="servo" -p TerminalRows=15 -p TerminalColumns=50 -e "roslaunch gmodctl servolocator.launch" &
mwait

#konsole -p tabtitle="rctl" -p TerminalRows=15 -p TerminalColumns=50 -e "roslaunch gmodctl rctl.launch rid:=1" &
#mwait

