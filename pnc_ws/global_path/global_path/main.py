import rclpy
from global_opt_settings import settings as settings
from global_opt_compiled import CompiledGlobalOpt
from rclpy.node import Node
import numpy as np
from feb_msgs import State
from feb_msgs import PathState

class GlobalPath(Node):
    def __init__(self): 
        super().__init__("global path")

        #Publishers
        self.pc_publisher = self.create_publisher(PathState, '/path/global', 10)
        self.cp_publisher = self.create_publisher(bool, '/path/finished', 10)
        
        #Subscribers
        self.pc_subscriber = self.create_subscription(    , '/slam/matched/global') #TODO: fill in type