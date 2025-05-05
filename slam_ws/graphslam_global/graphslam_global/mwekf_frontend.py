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
from time import perf_counter

from .GraphSLAMSolve import GraphSLAMSolve
from all_settings.all_settings import GraphSLAMSolverSettings as solver_settings
from all_settings.all_settings import GraphSLAMSettings as settings
from .utility_functions import compute_timediff, quat_to_euler, compute_delta_velocity, cartesian_to_polar
from .icp import run_icp, plot_icp
from .mwekf_backend import MWEKF_Backend

import matplotlib.pyplot as plt


class MWEKF(Node):
    def __init__(self, x0, num_cones):
        # ROS2 INTEGRATIONS
        
        self.mwekf =  MWEKF_Backend(x0, num_cones=10)

    def camera_sub(self, cones):
        pass
        #self.mwekf.update(cones, 0)

    def lidar_sub(self, cones):
        pass
        #self.mwekf.update(cones, 1)

    def wheelspeed_sub(self, velocity):
        self.mwekf.update(velocity, 3)

    def imu_callback(self, yaw):
        self.mwekf.update(yaw, 2)

    def mpc_input(self, accel, steering):
        self.mwekf.last_u[:] = [accel, steering, perf_counter()]