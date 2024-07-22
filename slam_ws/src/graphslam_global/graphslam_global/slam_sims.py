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

from feb_msgs.msg import State, FebPath, Map, Cones

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
            Cones,
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
def cartesian_to_polar(car_state: tuple[float, float], cone: tuple[float, float]):
    p_x = cone[0] - car_state[0]
    p_y = cone[1] - car_state[1]
    r = math.sqrt(p_x**2 + p_y**2)
    if (p_x == 0):
        angle = math.asin(p_y/r)
    else:
        angle = math.atan(p_y / p_x)
    if p_x < 0:
        angle = angle + math.pi
    return r, angle