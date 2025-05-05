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
from .mwekf_backend import MWEKF_Backend

import matplotlib.pyplot as plt


class MWEKF(Node):
    def __init__(self):
        # ROS2 INTEGRATIONS
        super().__init__('graphslam_global_node')
        
        self.mwekf =  MWEKF_Backend(num_cones=10)

    # Camera and Cones callbacks are gonna be tricky because we have to do icp and cone matching stuff
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
    
    def wheelspeed_sub(self, velocity):
        pass
        #self.mwekf.update(velocity, 3)

    def state_sub(self, msg):
        pass

    def imu_callback(self, yaw):
        pass
        #self.mwekf.update(yaw, 2)

    def mpc_input(self, msg):
        pass