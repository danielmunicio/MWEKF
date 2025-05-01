# Ros Imports 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header, Bool
from eufs_msgs.msg import WheelSpeedsStamped, CarState, ConeArrayWithCovariance, ConeWithCovariance
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Pose, Point, Point32
from sensor_msgs.msg import PointCloud, Imu
from feb_msgs.msg import State, FebPath, Map, Cones, ConesCartesian

# Python Libraries
import numpy as np


from .GraphSLAMSolve import GraphSLAMSolve
from all_settings.all_settings import GraphSLAMSolverSettings as solver_settings
from all_settings.all_settings import GraphSLAMSettings as settings
from .utility_functions import compute_timediff, quat_to_euler, compute_delta_velocity, cartesian_to_polar
from .icp import run_icp, plot_icp

import matplotlib.pyplot as plt


class MWEKF(Node):
    def __init__(self):
        # ROS2 INTEGRATIONS
        super().__init__('graphslam_global_node')
        
        self.slam = GraphSLAMSolve(**solver_settings)
        self.publish_to_rviz = settings.publish_to_rviz
        self.local_radius = settings.local_radius
        self.local_vision_delta = settings.local_vision_delta
        self.solve_by_time = settings.solve_by_time
        self.using_wheelspeeds = settings.using_wheelspeeds
        self.imu_direction = settings.forward_imu_direction
        if self.solve_by_time: 
            self.solve_frequency = settings.solve_frequency
        else:
            self.solve_distance = settings.solve_distance

        # Simulator Specific Settings
        self.using_simulator = settings.using_simulator
        if (self.using_simulator):
            self.using_ground_truth_cones = settings.using_ground_truth_cones
            self.using_ground_truth_wheelspeeds = settings.using_ground_truth_wheelspeeds
            self.using_ground_truth_state = settings.bypass_SLAM
        

        # SIMULATOR SPECIFIC SUBSCRIBERS 
        if (self.using_simulator):
            if (self.using_wheelspeeds):
                if (self.using_ground_truth_wheelspeeds):
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ground_truth/wheel_speeds', self.wheelspeed_sub_sim, 1)
                else: 
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ros_can/wheel_speeds', 1)
            
            self.state_subby = self.create_subscription(CarState, '/ground_truth/state', self.state_sub, 1,)
        else: 
            self.cones_sub = self.create_subscription(Cones, '/perception_cones', self.cones_callback, 1)
            if (self.using_wheelspeeds):
                self.wheelspeeds = self.create_subscription(Float64, '/odometry/wheelspeeds', self.wheelspeed_sub, 1)

        # IMU subscriber the same for both car and sim
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        # Cones subscribers
        self.cones_camera_sub = self.create_subscription(ConesCartesian, '/camera/cones', self.camera_cones_callback, 1)
        self.cones_lidar_sub = self.create_subscription(ConesCartesian, '/lidar/cones', self.lidar_cones_callback, 1)

        # PUBLISHERS
        # Publish the current vehicle's state: X, Y, Velo, Theta
        self.state_pub = self.create_publisher(State, '/slam/state', 1)

        # Publish the current map (GLOBAL_NODE, so this will send the whole map)
        self.global_map_pub = self.create_publisher(Map, '/slam/map/global', 1)

        self.local_map_pub = self.create_publisher(Map, '/slam/map/local', 1)

        if (self.publish_to_rviz):
            ##These are for Visuals in the SIM 
            self.cones_vis_pub = self.create_publisher(PointCloud, '/slam/conemap', 1)
            self.positionguess = self.create_publisher(PointCloud, '/slam/guessed_positions', 1)
            self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 1)

        self.recieved_camera = False
        self.recieved_lidar = False

        self.lidar_cones_map = None
        self.camera_cones_map = None
    def camera_cones_callback(self, msg: ConesCartesian):
        if not self.recieved_camera:
            self.camera_cones_map = np.vstack([msg.x, msg.y, msg.color])
            self.recieved_camera = True

        if self.recieved_camera and self.recieved_lidar:
            pass
    def lidar_cones_callback(self, msg: ConesCartesian):
        if not self.recieved_lidar:
            self.lidar_cones_map = np.vstack([msg.x, msg.y, msg.color])
            self.recieved_lidar = True
    
    def wheelspeed_sub_sim(self, msg):
        pass

    def state_sub(self, msg):
        pass

    def imu_callback(self, msg):
        pass