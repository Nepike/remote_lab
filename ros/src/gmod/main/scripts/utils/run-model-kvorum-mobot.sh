#!/bin/bash
#
# Запуск Kvorum-модели
# -- эмулятор локальной связи (lce)
# -- согласователь топиков (bridge)
# -- собственно управляющая программа
#

if [ $# -lt 1 ]
then
  echo -e "\n*** Usage is: $0 cellsize (in meters)\n"
  exit 0
fi

# --cellsize 0.3
cellsize=$1


USE_TMUX=0
SESSION="model-kvorum-mobot"

MODULE=main

SDIR=`pwd`
CFGDIR="$SDIR/config"

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

# Эмулятор локальной связи
command_lce="roslaunch gmodctl lce.launch"

# Согласователь
command_bridge="roslaunch gmodctl mobot_du_kvorum_bridge.launch rotvang:=10.0 rotvlin:=0.05 cspeed:=0.25 compass:=0 cellsize:=$cellsize"

# Основной скрипт
command_main="$SDIR/$MODULE.py --env $CFGDIR/env.ctl --map "$CFGDIR"/map.ctl --agn $CFGDIR/agents.ctl $3 $4 $5 $6 $7 $8 $9"
command_kill_nodes="rosnode kill mobot_du_kvorum_bridge_node lce_node"


if [ $USE_TMUX -eq 1 ];
then
    init_tmux
fi

exec_cmd 2 "lce"     "$command_lce"
exec_cmd 3 "bridge"  "$command_bridge"

if [ $USE_TMUX -eq 1 ];
then
    exec_cmd 1 "main" "$command_main ;  $command_kill_nodes ; tmux kill-session -t $SESSION"

    # Attach to created session
    tmux attach-session -t $SESSION
else
    $command_main
    if [ $? -ne 0 ]; then
        echo""
        echo "*** Error. Press Enter to continue..."
        read
    fi
    $command_kill_nodes
fi

