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

### RoboSense M1 LiDAR in ROS2
#### Install and compile driver
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

Then, go into the `config.yaml` file and configure the `msop_port` and `difop_port` to match that of the LiDAR, i.e.
``` yaml
msop_port: 6702     # Msop port of lidar
difop_port: 1210    # Difop port of lidar
```
Next, we need to clone the ROS2 packet message, which is done through (make sure you are still in `feb-system-integration/lidar_ws/src`)
``` bash
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
```
Now we are ready to compile the driver and source it.
``` bash
cd .. # Go back to /lidar_ws
colcon build
source install/setup.bash
```
#### Run the LiDAR node
To run the driver (which is required in order to use the LiDAR with ROS2), connect the LiDAR to your computer using the ethernet cable. Then, follow the steps of [this video](https://www.youtube.com/watch?v=Y3ZYh9g4TtU) after the 1:50 mark. The IP address that should be configured is the destination address of the LiDAR, i.e. `192.168.102.150`. You can install WireShark through
``` bash
sudo apt install wireshark
```
Note that the IP-address has to be set up manually each time your machine is rebooted.

Now, to run the driver, type
``` bash
ros2 launch rslidar_sdk start.py
```
This will open up `rViz2` with the `rs_lidar/points` topic being visualized
Note that this will not work in the `VSCode` terminal due to some unresolved issue (launching `rViz2` does not work in `VSCode`).

The repo of the driver with additional instruction can be found [here](https://github.com/RoboSense-LiDAR/rslidar_sdk/tree/main).

### Intel Realsense camera ROS2 driver 
The camera driver can simply be installed from the ROS2 servers using
``` bash
sudo apt install ros-humble-librealsense2*
```
Now, connect the camera to your computer using the USB cable and launch the camera driver using
``` bash
ros2 run realsense2_camera realsense2_camera_node
```
The camera image will now be published to the topic `realsense_camera/image_raw` TODO: check topic!!

