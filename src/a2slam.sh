#!/bin/bash

gnome-terminal --tab  -e "roscore" --tab -e "bash -c 'sleep 5 && rosparam set use_sim_time true && cd target_generator && source devel/setup.bash && source devel/setup.bash && roslaunch target_generator target_generator.launch'" --tab -e "bash -c 'sleep 5 && cd target_generator && source devel/setup.bash && rosrun target_generator target_generator'" --tab -e "bash -c 'sleep 5 && /usr/local/V-REP_PRO_EDU_V3_6_2_Ubuntu16_04/vrep.sh -s -q ~/target_generator/src/target_generator/scenes/sccene2.0_amb2.ttt'" --tab -e "bash -c 'sleep 5 && rosrun rviz rviz'"
