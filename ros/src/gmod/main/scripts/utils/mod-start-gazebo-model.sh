#!/bin/bash
#
# Запуск Gazebo-модели
# -- собственно модель
# -- ArUco-сервер
# -- серволокатор
# [-- монитор (ручное управление)]
#

USE_TMUX=1
SESSION="gazebo-model"

mwait()
{
  echo "Wait... "
  sleep 5
  echo "Done."
}

init_tmux()
{
    WINDOW_NAME="main"

    # Start tmux
    #tmux start-server

    # Create a new tmux session
    tmux new-session -d -s $SESSION -n $WINDOW_NAME

    # Create 4 panels
    tmux splitw -v -p 50

    tmux selectp -t 2

    tmux splitw -h -p 50

    tmux splitw -h -p 50

}

exec_cmd()
{
    # Exec command $3 in panel # $1
    if [ $USE_TMUX -eq 1 ];
    then
        tmux selectp -t $1
        tmux send-keys "$3" C-m
    else
        konsole -p tabtitle="$2" -p TerminalRows=15 -p TerminalColumns=50 -e "$3" &
    fi
    mwait
}

command_mobot="roslaunch mobot_du mobot.launch"
command_aruco="roslaunch gmodctl aruco.launch $1 $2 $3 $4"
command_servolocator="roslaunch gmodctl servolocator.launch"

if [ $USE_TMUX -eq 1 ];
then
    init_tmux
fi

exec_cmd 1 "gazebo_mobot_du" "$command_mobot"
exec_cmd 2 "aruco"           "$command_aruco"
exec_cmd 3 "servo"           "$command_servolocator"

if [ $USE_TMUX -eq 1 ];
then
    # Attach to created session
    tmux attach-session -t $SESSION
fi


# Ручное управление включать не будем.
# Если надо, то лучше сделать это вручную.
#konsole -p tabtitle="rctl" -p TerminalRows=15 -p TerminalColumns=50 -e "roslaunch gmodctl manctl.launch rid:=1" &
#mwait
