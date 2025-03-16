# Welcome to the Autonomous GitHub Repo!

## Table of Contents
1. [Installation](#installation)
    - [Required Environment](#required-environment)
    - [SSH Keys](#ssh-keys)
    - [Install CasADi, HSL Solvers, and Other Libraries](#install-casadi-hsl-solvers-and-other-libraries)
    - [Clone Repo to Your Machine](#clone-repo-to-your-machine)
    - [Install and Set Up Camera and LiDAR Drivers](#install-and-set-up-camera-and-lidar-drivers)
2. [Launching the Sensor Nodes](#launching-the-sensor-nodes)
    - [RoboSense M1 LiDAR](#robosense-m1-lidar)
    - [Intel Realsense Camera](#intel-realsense-camera)
3. [Table of Nodes](#table-of-nodes)
4. [Launchfiles](#launchfiles)

---
## Installation
Here, we will cover everything you will need to get going with coding for FEB Autonomous.


### Required environment
* Ubuntu 22.04 Jammy Jellyfish
    - [PC or non-M1/M2 Mac](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) (You need a USB stick for this)
    - [Using a VM](https://www.notion.so/Virtual-Machine-VM-ROS-Setup-Tutorial-350398bb897645a6a468814d065b1033) (Notion page, requires login)
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [COLCON](https://colcon.readthedocs.io/en/released/user/installation.html) (Make sure to copy the commands associated with "ROS2 repository")

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
This may take a while - expect 15-30 minutes. Make sure your computer doesn't die!
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

| Package `<package-name>`  | Node `<node-name>`     |
| ---                       |---                     |
|`mpc`                      |`mpc`                   |
|`local_path`               |`local_path`            |
|`global_path`              |`global_path`           |
|`graphslam_global`         |`graphslam_global`      |
|`microcontroller`          |`arduino_node`          |

## Launchfiles
A launchfile allows you to launch multiple nodes at the same time. The launch files we have are

* `ros2 launch racecar_bringup p131mks.launch.py` - Launches all nodes needed to run the sim (MPC, Local/Global Path, GraphSLAM). Will eventually have yaml file with sim configurations

* `perception_launch.py` - Launches the perception pipeline, i.e. camera, LiDAR and sensor fusion nodes. No sensor nodes are launched.

* `Full hardware launch` - to be added