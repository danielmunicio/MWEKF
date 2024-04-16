#!/bin/bash
source install/setup.bash
rm boobies.txt cones.txt local_path.txt mpc.txt out.txt path_values.txt
ros2 run fastslam fastslam &
ros2 run graphslam_global graphslam_global &
# ros2 run global_path global_path &
ros2 run local_path local_path &
ros2 run mpc mpc
