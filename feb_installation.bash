#!/bin/bash

# Variables
repo_name="feb-system-integration"
msop="6702"
difop="1210"

# Install LiDAR driver

# Make sure repo exists
if ! [ -d "$HOME/$repo_name" ]; then
    echo "(!) The repository '$repo_name' does not exist. Make sure it is cloned inside root directory"
    exit 1
fi

# Go to directory
cd ~/$repo_name

# Make sure it has not already been created
if [ -d "$HOME/$repo_name/lidar_ws" ]; then
    echo "(!) A LiDAR workspace already exists, aborting driver cloning"
    exit 1
fi

# # Create workspace
mkdir -p ./lidar_ws/src
cd $HOME/$repo_name/lidar_ws/src

# Clone the driver
echo "(*) Cloning SDK repo..."
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update


# Change compile method to COLCON in CMakeLists.txt
if [ -e "CMakeLists.txt" ]; then
  # Use sed to replace "CATKIN" with "COLCON" in line 13
  sed -i '13s/CATKIN/COLCON/' CMakeLists.txt
  echo "(*) Compilation method changed to COLCON in CMakeLists.txt"
else
  echo "(!) CMakeLists.txt not found in the current directory."
fi

# Check if package.xml and package_ros2.xml exist
if [ -e "package.xml" ] && [ -e "package_ros2.xml" ]; then
  # Replace the contents of package.xml with the contents of package_ros2.xml
  cat "package_ros2.xml" > "package.xml"
  echo "(*) Replaced the contents of package.xml with the contents of package_ros2.xml."
else
  echo "(!) Either package.xml or package_ros2.xml not found in the current directory."
fi



# Check if the config file exists in the "/config" directory
config_file="./config/config.yaml"
if [ -e "$config_file" ]; then
  # Use sed to modify lines 12 and 13
  sed -i "12s/msop_port: 6699/msop_port: $msop/" "$config_file"
  sed -i "13s/difop_port: 7788/difop_port: $difop/" "$config_file"
  echo "(*) Modified MSOP and DIFOP ports in $config_file."
else
  echo "(!) The config file $config_file not found in the /config directory."
fi

# Go back to /src
cd ..

# Clone rslidar_msg
echo "(*) Cloning RSLiDAR message repo..."
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git

# Go back to lidar_ws
cd ..
source /opt/ros/humble/setup.bash # Source ROS2
echo "(*) Compiling driver..."
colcon build # Build the packages
echo "(*) LiDAR driver successfully installed!"

cd $HOME

# Install camera driver
sudo apt install ros-humble-librealsense2*

# Define the command to add to ~/.bashrc
command_to_add="source ~/feb-system-integration/lidar_ws/install/setup.bash"

# Check if the command is already in ~/.bashrc
if ! grep -qF "$command_to_add" ~/.bashrc; then
    # If not found, add it to ~/.bashrc
    echo "$command_to_add" >> ~/.bashrc
    echo "Commands added to ~/.bashrc. It will be sourced each time you open a terminal."
else
    # If already found, inform the user
    echo "Commands already exist in ~/.bashrc. No changes made."
fi
