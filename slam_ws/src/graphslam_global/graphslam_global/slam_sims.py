import numpy as np
from numpy.linalg import *
# import matplotlib.pyplot as plt
# import csv
# import track_generator_leonid.track_main as trackmain
# import track_generator_leonid.settings as settings
# import graphslam_colorsolve as slamLib
from time import perf_counter
import math
# from numpy.random import random, randn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

from feb_msgs.msg import carstate, FebPath, Map, cones

class SLAMSims_In(Node):

    def __init__(self):
        
        # ROS2 INTEGRATIONS

        super().__init__('slam_sims_node')

        # NO SUBSCRIBERS

        # PUBLISHERS
        
        # Publish the vehicle's simulated imu readings
        self.imu_pub = self.create_publisher(
            Imu,
            '/odometry/Imu',
            1
        )

        # Publish the vehicle's simulated cone readings
        self.cone_pub = self.create_publisher(
            cones,
            '/perception/cones',
            1
        )

        # Map Integrations

        center_line, cones = self.generate_track(2)
        self.center_line = center_line
        self.cones = cones
        self.velocity_tracker = [0]
        self.vision_range = 30
        self.los = 30

        self.currIMU = Imu()
        self.currCones = cones()
        