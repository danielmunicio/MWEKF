# ROS Camera LIDAR Calibration Package

## Setup

Install dependencies.

```
sudo apt install ros-humble-camera-calibration
```

Run the following to clone the `lidar_camera_calibration` package in `ros_workspace/src` directory.

```
cd ~/ros_workspace/
colcon build
source install/setup.bash
```

Make sure you have the ROS bag file in `lidar_camera_calibration/bagfiles` folder. Then you can use the following launch files. This package assumes that the bag file has atleast the following topic names and message types by default, these can be modified in the launch scripts.

```
/rslidar_points (PointCloud2)
/sensors/camera/image_color (sensor_msgs/Image)
```

## Play ROS Bag File

This launch file will only play the rosbag record file.

```
ros2 launch lidar_camera_calibration play_rosbag.launch.py bagfile:=/path/to/file.bag
```

#### Camera Matrix

```
484.130454    0.000000  457.177461
  0.000000  484.452449  364.861413
  0.000000    0.000000    1.000000
```

#### Distortion Coefficients

```
-0.199619  0.068964  0.003371  0.000296  0.000000
```

## Update the ROS Bag File

`calibration_file.yaml` should contain the Intrinsic Parameters determined [here](https://github.com/FEBAutonomous/feb-system-integration/tree/integration/fusion/sensor_fusion/Intrinsic_Camera_Calibration)

```
ros2 run lidar_camera_calibration update_camera_info.py <original_file.bag> <calibration_file.yaml>
```

## Calibrate Camera-LiDAR Point Correspondences

This script will perform calibration using the matplotlib GUI to pick correspondences in the camera and the LiDAR frames. You first need to play the rosbag record in another terminal.

```
roslaunch lidar_camera_calibration play_rosbag.launch bagfile:=/path/to/file.bag
rosrun lidar_camera_calibration calibrate_camera_lidar.py --calibrate
```

Press [ENTER] to launch the GUIs and pick the corresponding points by selecting the four corner points of the checkerboard in both the camera and the LiDAR frames. You may update the point cloud field-of-view to display [here](https://github.com/heethesh/lidar_camera_calibration/blob/master/scripts/calibrate_camera_lidar.py#L232)  

OpenCV's PnP RANSAC + refinement using LM is used to find the rotation and translation transforms between the camera and the LiDAR. Since OpenCV's function rectifies the images internally, the 2D points are picked from the unrectified image. Additional, the `rectify` flag can be set to `True` while creating the GUI process to pick points from a rectified image.

**NOTE: The point files are appended and the extrinsics estimates are calculated and refined continuously using a RANSAC approach.**

**NOTE: To use `solvePnPRefineLM`, you need OpenCV >= 4.1.1, otherwise the LM pose refinement step will be skipped.**

The calibrated extrinsics are saved as following:
- `lidar_camera_calibration/calibration_data/lidar_camera_calibration/extrinsics.npz`
    - 'euler' : Euler Angles (RPY rad)
    - 'R'     : Rotation Matrix
    - 'T'     : Translation Offsets (XYZ m)

The following calibrated extrinsics were obtained:

#### Rotation Matrix
```
-9.16347982e-02  -9.95792677e-01  -8.74577923e-05
 1.88123595e-01  -1.72252569e-02  -9.81994299e-01
 9.77861226e-01  -9.00013023e-02   1.88910532e-01
```

#### Euler Angles (RPY rad)

```
-0.44460865  -1.35998386   2.0240699
```

#### Translation Offsets (XYZ m)

```
-0.14614803  -0.49683771  -0.27546327
```

## Display Camera-LiDAR Projection

This launch file will play the updated rosbag record, run `calibrate_camera_lidar.py` in projection mode and displays the LiDAR point cloud projected on to the image. A static transform is set up between the `world` and the `velodyne` frame which needs to be updates with the values above in the format `X Y Z Y P R` within the launch file. You may update the point cloud field-of-view to display [here](https://github.com/heethesh/lidar_camera_calibration/blob/master/scripts/calibrate_camera_lidar.py#L383).

```
roslaunch lidar_camera_calibration display_camera_lidar_calibration.launch
```
