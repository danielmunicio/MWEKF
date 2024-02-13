#!/bin/bash

# Variables
repo_name="feb-system-integration"
msop="6702"
difop="1210"
current_directory=$(basename "$(pwd)")

# Install LiDAR driver

# Make sure repo exists
if ! [ "$current_directory" == "$repo_name" ]; then
    echo "(!) The script has to be ran from folder '$repo_name'"
    exit 1
fi

# Make sure it has not already been created
if [ -d "lidar_ws" ]; then
    echo "(!) A LiDAR workspace already exists, aborting driver cloning"
    # exit 1
fi

# Apt and PIP installs
sudo apt-get update && sudo xargs apt-get -y install < ./requirements/apt.txt
pip install -r ./requirements/pip.txt

# Create workspace
mkdir -p lidar_ws/src
cd lidar_ws/src

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

cd ../..
source /opt/ros/humble/setup.bash # Source ROS2

echo "(*) Running colcon build"
colcon build

# Install intrinsic calibration package
sudo apt install ros-humble-camera-calibration-parsers
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-launch-testing-ament-cmake
sudo apt install ros-humble-camera-calibration

# Function to add a command to ~/.bashrc if it doesn't already exist
add_command_to_bashrc() {
  local command_to_add="$1"

  # Check if the command already exists in ~/.bashrc
  if grep -q "$command_to_add" "$HOME/.bashrc"; then
      echo "(i) Command '$command_to_add' already exists in ~/.bashrc"
  else
      # Add the command to ~/.bashrc
      echo -e "\n# Added by script\n$command_to_add" >> "$HOME/.bashrc"
      echo "(*) Command '$command_to_add' added to ~/.bashrc"
  fi
}

# Example commands to add
commands=(
  'source /opt/ros/humble/setup.bash'
)

# Loop through each command and add it to ~/.bashrc
for cmd in "${commands[@]}"; do
  add_command_to_bashrc "$cmd"
done

