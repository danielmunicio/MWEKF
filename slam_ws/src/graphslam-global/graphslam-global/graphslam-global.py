import numpy as np
from numpy.linalg import *
# import matplotlib.pyplot as plt
# import csv
# import track_generator_leonid.track_main as trackmain
# import track_generator_leonid.settings as settings
import graphslam_colorsolve as slamLib
from time import perf_counter
import math
# from numpy.random import random, randn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu


from feb_msgs.msg import State
from feb_msgs.msg import FebPath

class GraphSLAM_Global(Node):
    def __init__(self):

        # ROS2 INTEGRATIONS

        super().__init__('graphslam_global_node')

        # SUBSCRIBERS

        # Handle IMU messages for vehicle state
        self.imu_sub = self.create_subscription(
            Imu,
            '/odometry/Imu',
            self.imu_callback,
            1
        )

        # Handle new cone readings from perception
        self.cones_sub = self.create_subscription(
            List_of_Cones,
            '/perception/YOLO/Cones', # To be changed
            self.cones_callback,
            1
        )

        # PUBLISHERS
        
        # Publish the current vehicle's state: X, Y, Velo, Theta
        self.state_pub = self.create_publisher(
            State,
            '/slam/State',
            1
        )

        # Publish the current map (GLOBAL_NODE, so this will send the whole map)
        self.map_pub = self.create_publisher(
            Map, # To be created
            '/slam/Global-Map',
            1
        )

        # SLAM Initialization

        # Initializes a new instance of graphslam from the graphslam
        # Data Association Threshold is to be tweaked
        self.slam = slamLib.GraphSLAM(solver_type='qp', landmarkTolerance=4)
    
    
    """
    Function that takes in IMU messages and processes GraphSLAM based upon 
    Input: imu (Imu_msg)
    - geometry_msgs/Quarternion orientation
        - float64[9] orientation_covariance
    - geometry_msgs/Vector3 angular_velocity
        - float64[9] angular_velocity_covariance
    - geometry_msgs/Vector3 linear_acceleration
        - float64[9] linear_acceleration_covariance
    """
    def imu_callback(self, imu: Imu) -> None:
        # Dummy Function for now, need to decode the values (covariances)
        pass

    """
    Function that takes the list of cones, updates and solves the graph
    
    """
    def cones_callback(self, cones: List_of_Cones) -> None:
        # Dummy function for now, need to update graph and solve graph on each timestep
        pass

# For running node
def main(args=None):
    rclpy.init(args=args)
    graphslam_global_node = GraphSLAM_Global(Node)
    rclpy.spin(graphslam_global_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()