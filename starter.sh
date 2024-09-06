#!/bin/bash
source install/setup.bash;
ros2 run local_path compile_solver;
ros2 run fastslam fastslam > /dev/null &
ros2 run mpc mpc &
ros2 run global_path global_path &
ros2 run local_path local_path &
sleep 1;
ros2 run graphslam_global graphslam_global_fast
