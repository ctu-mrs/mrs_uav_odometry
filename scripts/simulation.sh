#!/bin/bash

SESSION_NAME=uav1
UAV_NAME=$SESSION_NAME
ESTIMATOR=$1
SENSORS=
LAUNCH_NODES=
WORLD='grass_plane'
# WORLD='worlds/cathedral.world'

if [ "$#" == 0 ]; then
  ESTIMATOR=gps
fi

if [ "$ESTIMATOR" == "optflow" ]; then
  SENSORS="$SENSORS --enable-bluefox-camera"
  LAUNCH_NODES='waitForOdometry; roslaunch mrs_optic_flow optic_flow.launch'
  WORLD='worlds/grass_plane.world'
# elif [ "$ESTIMATOR" == "gps" ]; then
#   SENSORS="$SENSORS --enable-rplidar"
#   LAUNCH_NODES='waitForOdometry; roslaunch hector_mapping uav.launch'
#   WORLD='worlds/cathedral.world'
elif [ "$ESTIMATOR" == "hector" ]; then
  SENSORS="$SENSORS --enable-rplidar"
  LAUNCH_NODES='waitForOdometry; roslaunch hector_mapping uav.launch'
  WORLD='worlds/cathedral.world'
elif [ "$ESTIMATOR" == "icp" ]; then
  SENSORS="$SENSORS --enable-rplidar"
  LAUNCH_NODES='waitForOdometry; roslaunch mrs_icp2d uav.launch'
  WORLD='worlds/cathedral.world'
elif [ "$ESTIMATOR" == "lidar" ]; then
  SENSORS="$SENSORS --enable-velodyne"
  LAUNCH_NODES='waitForOdometry; roslaunch aloam_velodyne velodyne_odometry_simulation.launch'
  WORLD='worlds/cathedral.world'
fi

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ATHAME_ENABLED=0; export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=127.0.0.1; export ODOMETRY_TYPE=$ESTIMATOR"

# define commands
# 'name' 'command'
input=(
  'Roscore' 'roscore
'
  'Gazebo' "waitForRos; roscd mrs_simulation; roslaunch mrs_simulation simulation.launch gui:=true debug:=false world_name:=$WORLD --wait
"
  'Spawn' "waitForSimulation; rosrun mrs_simulation spawn 1 --run --delete --$UAV_TYPE --enable-rangefinder --enable-ground-truth $SENSORS --file ~/mrs_workspace/src/uav_core/ros_packages/mrs_odometry/config/init_pose/init_pose.csv
"
  'MRS_control' "waitForOdometry; roslaunch mrs_uav_general core.launch
"
  'Bumper' "waitForOdometry; roslaunch mrs_bumper bumper.launch
"
  'Localization' "waitForOdometry; $LAUNCH_NODES
"
  "PrepareUAV" "waitForControl; rosservice call /$UAV_NAME/control_manager/motors 1; rosservice call /$UAV_NAME/mavros/cmd/arming 1; rosservice call /$UAV_NAME/mavros/set_mode 0 offboard; rosservice call /$UAV_NAME/uav_manager/takeoff;
"
  'ChangeEstimator' "rosservice call /$UAV_NAME/odometry/change_estimator_type_string GPS"
  'ChangeHdgEstimator' "rosservice call /$UAV_NAME/odometry/change_hdg_estimator_type_string GPS"

  'Constraints' "rosservice call /$UAV_NAME/constraint_manager/set_constraints darpa"
  'GoTo' "rosservice call /$UAV_NAME/control_manager/goto \"goal: [0.0, 0.0, 3.0, 0.0]\""
  'GoToRelative' "rosservice call /$UAV_NAME/control_manager/goto_relative \"goal: [0.0, 0.0, 0.0, 0.0]\""
  'GoFcu' "rosservice call /$UAV_NAME/control_manager/goto_fcu \"goal: [0.0, 0.0, 0.0, 0.0]\""
  'RVIZ' "waitForOdometry; roscd mrs_odometry; ./scripts/change_uav.sh $UAV_NAME; rosrun rviz rviz -d rviz/odometry.rviz
  "
  'rviz_interface' "waitForOdometry; roslaunch mrs_rviz_interface mrs_rviz_interface.launch
"
  'Juggler' "waitForOdometry; roscd mrs_odometry; ./scripts/change_uav.sh $UAV_NAME; i3 workspace "9"; rosrun plotjuggler PlotJuggler -l plot_juggler/odometry.xml"
  'reconfigure' " waitForOdometry; rosrun rqt_reconfigure rqt_reconfigure"
  # 'Layout' "waitForControl; i3 workspace "9"; ~/.i3/layout_manager.sh rviz_rqt_juggler
  # "
  'Camera_follow' "waitForOdometry; gz camera -c gzclient_camera -f uav1
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
