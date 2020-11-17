 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)
## You need to create a "config.sh file in the same folder defining your values for these variables"
source config.sh
export METACONTROL_WS_PATH


####
#  Default values, set if no parameters are given
####
# declare -a configs=("dwa_v0_a0_b0" "dwa_v0_a1_b0" "dwa_v0_a1_b1" "dwa_v0_a0_b1" "dwa_v1_a0_b0" "dwa_v1_a1_b0" "dwa_v1_a1_b1" "dwa_v1_a0_b1" "teb_v0_a0_b0" "teb_v0_a1_b0" "teb_v0_a1_b1" "teb_v0_a0_b1" "teb_v1_a0_b0" "teb_v1_a1_b0" "teb_v1_a1_b1" "teb_v1_a0_b1" )
declare -a configs=("dwa_v2_a0_b0" "dwa_v2_a1_b0" "dwa_v2_a1_b1" "dwa_v2_a0_b1" "dwa_v1_a0_b0" "dwa_v1_a1_b0" "dwa_v1_a1_b1" "dwa_v1_a0_b1" "teb_v2_a0_b0" "teb_v2_a1_b0" "teb_v2_a1_b1" "teb_v2_a0_b1" "teb_v1_a0_b0" "teb_v1_a1_b0" "teb_v1_a1_b1" "teb_v1_a0_b1" )
# declare -a configs=("dwa_v1_a0_b0" "dwa_v1_a1_b0")


declare exp="f1"
declare x_start="0.0"
declare y_start="0.0"
declare yaw_start="0.0"
# Relative. The robot expects to start in 0,0,0
# declare x_goal="28.0"
# declare y_goal="-3.5"
declare x_goal="21.0"
declare y_goal="0.0"
declare yaw_goal="0.0"
# declare x_goal="3.0"

declare launch_reconfiguration="true"
declare qa_updater="false"
declare modification="0"
declare close_reasoner_terminal="true"

declare nav_profile="dwa_v1_a0_b0"
declare nfr_safety="0.6"
declare reasoning_rate="0.6"

declare bagname="metacontrol_"


while getopts ":i:g:n:r:o:p:e:s:c:b:m:" opt; do
	case $opt in
		i) init_position="$OPTARG"
		;;
		g) goal_position="$OPTARG"
		;;
		n) nav_profile="$OPTARG"
		;;
		r) launch_reconfiguration="$OPTARG"
		;;
		o) obstacles="$OPTARG"
		;;
		p) increase_power="$OPTARG"
		;;
		e) nfr_energy="$OPTARG"
		;;
		s) nfr_safety="$OPTARG"
		;;
		c) close_reasoner_terminal="$OPTARG"
		;;
		b) bagname="$OPTARG"
		;;
		m) modification="$OPTARG"
 		;;
		\?) echo "Invalid option -$OPTARG" >&2
			usage
		;;
	esac
done


if [ "$modification" = "0" ] ; then
	qa_updater="false"
fi
if [ "$modification" = "1" ] ; then
	qa_updater="true"
fi
if [ "$modification" = "2" ] ; then
	qa_updater="true"
fi

echo "bagname = $bagname"

wait_for_gzserver_to_end () {

	for t in $(seq 1 100)
	do
		if test -z "$(ps aux | grep gzserver | grep -v grep )"
		then
			# echo " -- gzserver not running"
			break
		else
			echo " -- gzserver still running"
		fi
		sleep 1
	done
}
kill_running_ros_nodes () {
	# Kill all ros nodes that may be running
	for i in $(ps aux | grep ros | grep -v grep | awk '{print $2}')
	do
		echo "kill -2 $i"
		kill -2 $i;
	done
	sleep 1
}
kill_robot_nodes () {
	gnome-terminal --window --geometry=80x24+10+10 -- bash -c "
	rosnode kill move_base ekf_localization robot_state_publisher controller_spawner twist_mux twist_marker_server record_bag_node fake_localization /bluetooth_teleop/joy_node /bluetooth_teleop/teleop_twist_joy;
	rosnode kill mros1_reasoner_node
	rosservice call gazebo/delete_model '{model_name: /}'
	exit"
}

# kill_robot_nodes
# sleep 1

gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash;
roslaunch simulation_tests spawn_boxer.launch x:=${x_start} y:=${y_start} yaw:=${yaw_start};
exit"

echo "Launching: Jasper metacontrol.launch"
gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash;
rosparam set /desired_configuration \"$nav_profile\";
rosparam set /nfr_safety \"$nfr_safety\";
rosparam set /reasoning_rate \"$reasoning_rate\";
rosparam set /modification \"$modification\";
roslaunch thesis_experiments Jasper_metacontrol.launch nav_profile:=$nav_profile bag_store_path:='$METACONTROL_WS_PATH/src/thesis_experiments/bags/' bagname:=$bagname;
if [ '$close_reasoner_terminal' = false ] ; then read -rsn 1 -p 'Press any key to close this terminal...' echo; fi
exit"
if [ "$qa_updater" = true ] ; then
	echo "Launching: qa updater"
	gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash;
	echo 'Launching: qa updater'
	rosrun mros_quality_models quality_update.py;
	echo 'mros qa updater finished';
	exit"
	sleep 2
	for config in ${configs[@]} ; do
		bash -c "
		rosservice call /load_safety_model \"name: '$config'\""
	done
fi
if [ "$launch_reconfiguration" = true ] ; then
	echo "Launching: mros reasoner"
	gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash;
	roslaunch mros1_reasoner run.launch model:=$(rospack find mros1_reasoner)/scripts/kb_nav_quality_v2.owl desired_configuration:=$nav_profile nfr_safety:=$nfr_safety;
	echo 'mros reasoner finished';
	if [ '$close_reasoner_terminal' = false ] ; then read -rsn 1 -p 'Press any key to close this terminal...' echo; fi
	exit"
fi

# echo "Start navigation manager"
bash -c "
cd $METACONTROL_WS_PATH/src/thesis_experiments/scripts;
./navigation_manager.py $x_goal $y_goal $yaw_goal;
exit;"

echo "Experiments finished!!"

# kill_robot_nodes
# echo "Running log and stop simulation node"
# bash -ic "source $METACONTROL_WS_PATH/devel/setup.bash;
# roslaunch metacontrol_experiments stop_simulation.launch obstacles:=$obstacles goal_nr:=$goal_position increase_power:=$increase_power record_bags:=$record_rosbags;
# exit "
# echo "Simulation Finished!!"

# Check that there are not running ros nodes
# kill_running_ros_nodes
# Wait for gazebo to end
# wait_for_gzserver_to_ends
