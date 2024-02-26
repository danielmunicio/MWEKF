import rclpy
from global_opt_settings import settings as settings
from global_opt_compiled import CompiledGlobalOpt
from rclpy.node import Node
import numpy as np
from global_opt_settings import settings
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

        ## PSUEDOCODE
        # input: cones from slam
        blue_cones = ... #Message from /slam/matched/global
        yellow_cones = ... #Message from /slam/matched/global
        g = CompiledGlobalOpt(**settings)
        
        g.construct_solver()
        g.load_solver()

        left_points, right_points = yashs_algorithm(blue_cones, yellow_cones)
        res = g.solve(left_points, right_points)
        states, controls = g.to_constant_tgrid(**res)
        bools = []
        #for i in states, determine whether each state is true or false and add it to bools
        #publish bools to /global/finished
    
        

        publish(states)
