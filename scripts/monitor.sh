#!/bin/bash

UAV_NAME=uav$(~/.scripts/get_uav_number.sh)
SESSION_NAME=monitor_$UAV_NAME

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ATHAME_ENABLED=0"

# define commands
# 'name' 'command'
input=(
  'RVIZ' "waitForRos;roscd mrs_odometry; ./scripts/change_uav.sh $UAV_NAME; nice -n 15 rosrun rviz rviz -d rviz/odometry.rviz
  "
  'Juggler' "waitForRos; sleep 1; roscd mrs_odometry; ./scripts/change_uav.sh $UAV_NAME; i3 workspace "9"; rosrun plotjuggler PlotJuggler -l plot_juggler/odometry.xml
  "
  'reconfigure' " waitForRos; rosrun rqt_reconfigure rqt_reconfigure
  "
  'Layout' "waitForRos; i3 workspace '9'; sleep 8;  ~/.i3/layout_manager.sh rviz_rqt_juggler
  "
)

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  SESSION_NAME="$(tmux display-message -p '#S')"
fi

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}" 
	((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
	tmux new-window -t $SESSION_NAME:$(($i+10)) -n "${names[$i]}"
done

sleep 4

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
	tmux send-keys -t $SESSION_NAME:$(($i+10)) "${pre_input};
${cmds[$i]}"
done

sleep 4

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear
