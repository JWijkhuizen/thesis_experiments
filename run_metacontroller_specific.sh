 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)
## You need to create a "config.sh file in the same folder defining your values for these variables"
source config.sh
export METACONTROL_WS_PATH


####
#  Default values, set if no parameters are given
####
declare -a configs=("dwa_v1_a0_b0")

declare reconfiguration="true"

declare qa_updater="true"

# declare -a scenarios=("0 0.4 0" "0 0 0.4" "0 0.4 0.4")
# declare -a scenarios=("0 0.4 0.4")
# declare -a scenarios=("S1" "S2" "S3")


# declare -a mods=("0" "1" "2")
# declare -a mods=("0")

# declare -a runs=("1" "2" "3" "4" "5")
declare -a runs=("1")

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
	rosservice call gazebo/delete_model '{model_name: /}'
	rosnode kill /move_base 
	rosnode kill /ekf_localization
	rosnode kill /robot_state_publisher /controller_spawner
	rosnode kill /twist_mux /twist_marker_server 
	rosnode kill /record_bag_node 
	rosnode kill /fake_localization 
	rosnode kill /bluetooth_teleop/joy_node /bluetooth_teleop/teleop_twist_joy;
	rosnode kill /reasoner
	exit"
}
start_simulation () {
	echo ""
	echo "Start a new simulation"
	echo ""
	echo "Launch roscore"
	gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash; roscore; exit"

	sleep 3

	echo "Launching: Gazebo empty world"
	gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash;
	roslaunch gazebo_ros empty_world.launch;
	exit"

	echo "Spawn ahxl and obstacles"
	bash -c "
	cd $METACONTROL_WS_PATH/src/ahxl_gazebo/scripts;
	./generate_supermarket.py;
	./spawn_obstacles.py  $1;
	exit;"
}



declare fail=2

# let runmax=${#scenarios[@]}\*${#configs[@]}\*${#mods[@]}\*${#runs[@]}
# declare runid=1

# metacontrol_M0_dwa_v1_a0_b0_S2_r1_2020-10-26-07-30-14.bag
# metacontrol_M2_dwa_v1_a0_b0_S2_r2_2020-10-26-08-39-40.bag
# metacontrol_M2_dwa_v1_a0_b0_S2_r5_2020-10-26-08-45-37.bag


# metacontrol_M0_dwa_v1_a0_b0_S3_r2_2020-10-26-10-00-00.bag
# metacontrol_M0_dwa_v1_a0_b0_S3_r3_2020-10-26-10-00-58.bag
# metacontrol_M0_dwa_v1_a0_b0_S3_r4_2020-10-26-10-01-57.bag
# metacontrol_M0_dwa_v1_a0_b0_S3_r5_2020-10-26-10-02-56.bag

# metacontrol_M2_dwa_v1_a0_b0_S3_r1_2020-10-26-10-19-33.bag
# metacontrol_M2_dwa_v1_a0_b0_S3_r2_2020-10-26-10-22-27.bag
# metacontrol_M2_dwa_v1_a0_b0_S3_r3_2020-10-26-10-26-45.bag
# metacontrol_M2_dwa_v1_a0_b0_S3_r4_2020-10-26-10-29-20.bag
# metacontrol_M2_dwa_v1_a0_b0_S3_r5_2020-10-26-10-31-55.bag

declare -a scenarios=("S3")
declare -a mods=("2")

for scenario in ${scenarios[@]} ; do
	# Check that there are not running ros nodes
	kill_running_ros_nodes
	# If gazebo is running, it may take a while to end
	wait_for_gzserver_to_end
	# Start simulation
	start_simulation $scenario
	for config in ${configs[@]} ; do
		for mod in ${mods[@]} ; do

			if [ $mod -eq "1" ] ; then
				runs=("1" "2")
			fi
			if [ $mod -eq "2" ] ; then
				runs=("4")
			fi

			for run in ${runs[@]} ; do
				# echo "Simulation run ${runid}/${runmax}"
				echo "M${mod}, ${scenario}, run${run}"
				bagname="metacontrol_M${mod}_${config}_${scenario}_r${run}"
				# bagname="test"
				fail=1
				while [ $fail -ne 0 ] ; do
					bash -ic "./run_single_sim.sh -n $config -r $reconfiguration -m $mod -b $bagname;
					exit"
					sleep 1
					bash -ic "./check_fail.py ;
					exit"
					fail=$?
					if [ $fail -ne 0 ]; then
						echo "${fail}: Failed run, try again. Reasoner failed"
					fi
					kill_robot_nodes
					sleep 1
				done
				# runid=$((runid+1))
			done
		done
		# echo "End of run"
	done
done
kill_running_ros_nodes
