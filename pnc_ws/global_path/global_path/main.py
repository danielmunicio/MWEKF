from typing import List
import rclpy
from global_opt_compiled import CompiledGlobalOpt
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, String
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import sys
from feb_msgs import State


class GlobalPath(Node):
    def __init__(self): 
        super().__init__("global path")

        #Publishers
        self.pc_publisher = self.create_publisher(State, '/path/global', 10)
        self.cp_publisher = self.create_publisher(bool, '/path/finished', 10)
        
        #Subscribers
        self.pc_subscriber = self.create_subscription(State, '/slam/matched/global')
        
