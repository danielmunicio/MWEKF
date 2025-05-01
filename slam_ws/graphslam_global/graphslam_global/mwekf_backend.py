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

class MWEKF_Backend():
    def __init__(self):
        # Dynamics Model

        self.Q = None
        self.R = None
        self.P = None
        self.C = None

        self.last_time = None



    #################
    # Different Measurement Models 
    #################

    def approximate_cones_measurement(self):
        """
        Cones measurement comes in N x 2 of [x y]
        Rotation Matrix is [[cos(theta), sin(theta)], [-sin(theta), cos(theta)] 
        Translation is Nx2 of [x y] or just broadcast or something
        """
        pass

    def approximate_imu_measurement(self):
        pass

    def approximate_wheelspeed_measurement(self):
        pass


    ################
    # State Estimation based on Dynamics
    # Might add multiple state estimation steps based on dynamics before doing update, to fully utilize MPC speed
    ################
    def g(self):
        pass

    def approximate_A(self):
        pass


    def update(self, measurement, measurement_type):
        """
        Updates EKF, based on measurement
        Measurement Types: 
        0 = cones_camera - Nx2 Array of [x y]
        1 = cones_lidar - Nx2 Array of [x y]
        2 = IMU - idk yet
        3 = Wheelspeeds - velocity measurement ? 
        """
        pass