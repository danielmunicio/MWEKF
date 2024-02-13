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

### Install CasADi, HSL solvers, and other silly libraries
this may take a while - expect 15-30 minutes. Make sure your computer doesn't die!
There will likely be a bunch of warnings. Don't cancel the execution. If it doesn't work, it's easier to diagnose if we know what it did.
```bash
cd casadi_src; . casadi_installer.sh;
```

### Clone repo to your machine
``` bash
cd ~/
git clone git@github.com:FEBAutonomous/feb-system-integration.git
```
If it says you don't have git (perhaps if this is a fresh install), run these commands:
```
sudo apt update; sudo apt upgrade
sudo apt install git curl
```
### Install and set up camera and LiDAR drivers
Go into the repo and run the installation script.
``` bash
cd ~/feb-system-integration
bash feb_installation.bash
```
Then, to configure the LiDAR driver, **do not connect the LiDAR to your computer yet**, but connect your ethernet dongle if you do not have an ethernet port. Run the network configuration script.
``` bash
cd ~/feb-system-integration
bash feb_lidar_setup.bash
```
In case you switch to a different dongle, the script will create a new connection profile if you run it again. When the connection profile has been created, you can connect the LiDAR to your computer. Make sure the green LED on the ethernet port is blinking at a constant high frequency; it means that data is being received by your computer.

## Launching the sensor nodes
### RoboSense M1 LiDAR
To run the driver (which is required in order to use the LiDAR with ROS2), connect the LiDAR to your computer using the ethernet cable and run the node.
``` bash
ros2 launch rslidar_sdk start.py
```
This will open up `rViz2` with the `rs_lidar/points` topic being visualized. Note that this will not work in the `VSCode` terminal due to some unresolved issue (launching `rViz2` does not work in `VSCode`).

The repo of the driver with additional instruction can be found [here](https://github.com/RoboSense-LiDAR/rslidar_sdk/tree/main).

### Intel Realsense camera
Connect the camera to your computer using the USB cable and launch the camera driver using
``` bash
ros2 run realsense2_camera realsense2_camera_node
```
In this case, no `rViz2` window will be opened.

## Table of nodes
To run node with name `<node-name>`, open a terminal and go to the `<ws-name>` workspace of the node and source it.
``` bash
cd ~/feb-system-integration/<ws-name>
# Build the workspace here if you haven't!
source install/setup.bash
```
Remember that you must have built the workspace in order to do this.

To run the node, type
``` bash
ros2 run <package-name> <node-name>
```
where `<package-name>` is the package where the node is located. All nodes, their packages, and workspaces are presented in the table below.

| Workspace `<ws-name>` | Package `<package-name>`  | Node `<node-name>`|
|---                    |---                        |---                |
|`perception_ws`        |`auto_calibration`         |`main`             |
|`perception_ws`        |`camera_perception`        |`yolov8_node`      |
|`perception_ws`        |`lidar_perception_cpp`     |`lidar_node_cpp`   |
|`perception_ws`        |`sensor_fusion`            |`fusion_node`      |

## Launchfiles
A launchfile allows you to launch multiple nodes at the same time. The launch files we have are

* `perception_launch.py` - Launches the perception pipeline, i.e. camera, LiDAR and sensor fusion nodes. No sensor nodes are launched.

To launch, the terminal you are running from first has to be sourced properly. Make sure all workspaces
where the nodes you will use are located are built with `colcon`. Then, source the terminal first by sourcing `comm_ws` to create an underlay (all our lauchfiles require this).
```bash
source ~/feb-system-integration/comm_ws/install/setup.bash
```
Repeat this with all required workspaces, i.e.
```bash
source ~/feb-system-integration/<required_ws>/install/setup.bash
```
where `<required_ws>` is a required workspace. Now, you can go into the `launch` folder and use `ros2 launch`, e.g.
``` bash
cd ~/feb-system-integration/launch
ros2 launch <file-name>
```
where `<file-name>` is one of the launchfiles listed above.
