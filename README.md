# Welcome to the FEB (soon to be) official GitHub Repo!

## Installation
Here, we will cover everything you will need to get going with coding for FEB Autonomous.


### Recommended environment
* Ubuntu 22.04 Jammy Jellyfish (TODO: Add internal link to installations for different OS)
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

### SSH keys
To be able to push your code to our repo, you will need to generate an SSH key.
``` bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```
The `"your_email@example.com"` can be replaced with any identifier you see fit. You will then be presented with some options and default shold work fine; press `ENTER` to accept the default settings.

Copy the contents of
``` bash
cat ~/.ssh/id_ed25519.pub
```
and add the key under `Settings > SSH and GPG keys` on GitHub.

### Clone repo to your machine
``` bash
cd ~/
git clone git@github.com:FEBAutonomous/feb-system-integration.git
```

### RoboSense M1 LiDAR ROS2 driver
First, we will need to create a ROS2 workspace to compile the driver. (DON'T FORGET TO CHANGE NAME OF REPO)
``` bash
cd ~/feb-system-integration
mkdir -p ./lidar_ws/src
cd ./lidar_ws/src
```
Next, clone the drivers from GitHub into the `src` folder of the workspace.
``` bash
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
```
Now, open the `CMakeLists.txt` file inside the `rslidar_sdk` folder and set the variable `COMPILE_METHOD` to `COLCON`.
``` cmake
#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```
Now, we want to install the correct dependencies. Go into the file `package_ros2.xml` and copy **everything** from it (e.g. by doing `Ctrl+A Ctrl+C`) and pasting it (`Ctrl+V`) into the file `package.xml`. Note that you should **fully** replace the contents in this file with what you're pasting.

Next, we need to clone the ROS2 packet message, which is done through (make sure you are still in `feb-system-integration/lidar_ws/src`)
``` bash
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
```
Now we are ready to compile the driver.
``` bash
cd .. # Go back to /lidar_ws
colcon build
source install/setup.bash
```
To run the driver (which is required in order to use the LiDAR with ROS2), type
``` bash
ros2 launch rslidar_sdk start.py
```
The repo of the driver with additional instruction can be found [here](https://github.com/RoboSense-LiDAR/rslidar_sdk/tree/main).




## Perception workspace

### RoboSense M1 LiDAR setup

How to set up with ROS2 driver when connected can be found [here](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/06_how_to_decode_online_lidar.md)