 #!/bin/bash

## Define path for workspaces (needed to run reasoner and metacontrol_sim in different ws)
## You need to create a "config.sh file in the same folder defining your values for these variables"
source config.sh
export METACONTROL_WS_PATH


####
#  Default values, set if no parameters are given
####
# declare -a configs=("dwa_v0_a0_b0" "dwa_v0_a1_b0" "dwa_v0_a1_b1" "dwa_v0_a0_b1" "dwa_v1_a0_b0" "dwa_v1_a1_b0" "dwa_v1_a1_b1" "dwa_v1_a0_b1" "teb_v0_a0_b0" "teb_v0_a1_b0" "teb_v0_a1_b1" "teb_v0_a0_b1" "teb_v1_a0_b0" "teb_v1_a1_b0" "teb_v1_a1_b1" "teb_v1_a0_b1" "dwa_v2_a0_b0" "dwa_v2_a1_b0" "dwa_v2_a1_b1" "dwa_v2_a0_b1" "teb_v2_a0_b0" "teb_v2_a1_b0" "teb_v2_a1_b1" "teb_v2_a0_b1")
declare -a configs=("dwa_v1_a0_b0" "dwa_v1_a1_b0" "dwa_v1_a1_b1" "dwa_v1_a0_b1" "teb_v1_a0_b0" "teb_v1_a1_b0" "teb_v1_a1_b1" "teb_v1_a0_b1" "dwa_v2_a0_b0" "dwa_v2_a1_b0" "dwa_v2_a1_b1" "dwa_v2_a0_b1" "teb_v2_a0_b0" "teb_v2_a1_b0" "teb_v2_a1_b1" "teb_v2_a0_b1")
# declare -a configs=("dwa_v1_a0_b0" "teb_v1_a0_b0")


declare reconfiguration="false"

# declare -a scenarios=("0 0.4 0" "0 0 0.4" "0 0.4 0.4")
declare -a scenarios=("V1")

declare -a runs=("1")

let runmax=${#configs[@]}
declare runid=0
declare bagname="benchmark"
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
	rosnode kill move_base robot_state_publisher controller_spawner twist_mux
	rosnode kill ekf_localization twist_marker_server 
	rosnode kill record_bag_node 
	rosnode kill /bluetooth_teleop/joy_node /bluetooth_teleop/teleop_twist_joy;
	rosnode kill /fake_localization
	exit"
}
start_simulation () {
	echo ""
	echo "Start a new simulation"
	echo ""
	echo "Launch roscore"
	gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash; roscore; exit"

	sleep 3

	echo "Launching: Simulation tests world.launch"
	gnome-terminal --window --geometry=80x24+10+10 -- bash -c "source $METACONTROL_WS_PATH/devel/setup.bash;
	roslaunch simulation_tests world.launch;
	exit"

	echo "Spawn ahxl and obstacles"
	bash -c "
	cd $METACONTROL_WS_PATH/src/ahxl_gazebo/scripts;
	./generate_supermarket.py;
	./spawn_obstacles.py $1;
	exit;"
}



let runmax=${#scenarios[@]}\*${#configs[@]}\*${#runs[@]}
declare runid=1

declare fail=2


for scenario in ${scenarios[@]} ; do
	# Check that there are not running ros nodes
	kill_running_ros_nodes
	# If gazebo is running, it may take a while to end
	wait_for_gzserver_to_end
	# Start simulation
	start_simulation $scenario
	for config in ${configs[@]} ; do
		for run in ${runs[@]} ; do
			echo "${runid}/${runmax}: ${config}, ${scenario}, run${run}"
			# bagname="benchmark_${config}_${scenario}_r${run}"
			bagname="validation_${config}"
			fail=1
			while [ $fail -ne 0 ] ; do
				echo "${runid}/${runmax}"
				echo "$scenario"
				bash -ic "./run_single_sim.sh -n $config -r $reconfiguration -b $bagname;
				exit;"
				fail=$?
				if [ $fail -ne 0 ]; then
					echo "${fail}: Failed run, try again"
				fi
				kill_robot_nodes
				sleep 1
			done
			runid=$((runid+1))
		done
	done
done

kill_running_ros_nodes




