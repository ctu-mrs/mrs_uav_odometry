#!/bin/bash

SESSION_NAME=uav1
UAV_NAME=$SESSION_NAME

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ATHAME_ENABLED=0"

# define commands
# 'name' 'command'
input=(
  'Roscore' 'roscore
'
  'Gazebo' "waitForRos; roslaunch simulation church.launch gui:=true
"
  'Spawn' "waitForSimulation; spawn 1 --run --delete --enable-rangefinder --enable-ground-truth --enable-rplidar --enable-bluefox-camera --file ~/mrs_workspace/src/uav_core/ros_packages/mrs_odometry/config/init_pose/init_pose.csv
"
  'MRS_control' "waitForOdometry; roslaunch mrs_uav_manager simulation_f550_hector.launch
"
  'OpticFlow' "waitForOdometry; roslaunch mrs_optic_flow simulation_nodelet_cpu.launch
"
  'Hector Slam' "waitForOdometry; roslaunch hector_mapping uav.launch
"
  'Lidar Stab' "waitForOdometry; roslaunch lidar_stabilization simulation.launch
"
  "PrepareUAV" "waitForControl; rosservice call /$UAV_NAME/mavros/cmd/arming 1; rosservice call /$UAV_NAME/control_manager/motors 1; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard; rosservice call /$UAV_NAME/uav_manager/takeoff;
"
  'Camera_follow' "waitForOdometry; gz camera -c gzclient_camera -f uav1
"
  'ChangeEstimator' "rosservice call /$UAV_NAME/odometry/change_estimator_type_string HECTOR"

  'Constraints' "rosservice call /$UAV_NAME/constraint_manager/set_constraints darpa"
  'GoTo' "rosservice call /$UAV_NAME/control_manager/goto \"goal: [0.0, 0.0, 3.0, 0.0]\""
  'GoToRelative' "rosservice call /$UAV_NAME/control_manager/goto_relative \"goal: [0.0, 0.0, 0.0, 0.0]\""
  'RVIZ' "waitForOdometry; roscd mrs_odometry; ./scripts/change_uav.sh $UAV_NAME; nice -n 15 rosrun rviz rviz -d rviz/odometry.rviz
  "
  'Juggler' "waitForOdometry; roscd mrs_odometry; ./scripts/change_uav.sh $UAV_NAME; i3 workspace "9"; rosrun plotjuggler PlotJuggler -l plot_juggler/odometry.xml
  "
  'reconfigure' " waitForOdometry; rosrun rqt_reconfigure rqt_reconfigure
  "
  'Layout' "waitForControl; i3 workspace "9"; ~/.i3/layout_manager.sh simulation
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
