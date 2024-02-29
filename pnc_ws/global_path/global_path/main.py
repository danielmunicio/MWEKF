import rclpy
from global_opt_settings import settings as settings
from global_opt_compiled import CompiledGlobalOpt
from rclpy.node import Node
import numpy as np
from global_opt_settings import GlobalOptSettings as settings
from feb_msgs import State
from feb_msgs import PathState
from feb_msgs import Cones
from ConeOrdering import ConeOrdering
class GlobalPath(Node):
    def __init__(self): 
        super().__init__("global_path")

        #Publishers
        self.pc_publisher = self.create_publisher(PathState, '/path/global', 10)
        self.cp_publisher = self.create_publisher(bool, '/path/finished', 10)
        
        #Subscribers
        self.pc_subscriber = self.create_subscription(Cones, '/slam/matched/global', self.listener_cb, 10) #TODO: fill in type

        ## PSUEDOCODE
        # input: cones from slam
        # blue_cones = ... #Message from /slam/matched/global
        # yellow_cones = ... #Message from /slam/matched/global
        # self.g = CompiledGlobalOpt(**settings)
        # self.g.load_solver()

        # left_points, right_points = yashs_algorithm(blue_cones, yellow_cones)
        # res = g.solve(left_points, right_points)
        # states, controls = g.to_constant_tgrid(**res)
        # bools = []
        #for i in states, determine whether each state is true or false and add it to bools
        #publish bools to /global/finished
    
        

        # publish(states)

    def listener_cb(self, msg):
        left, right = ConeOrdering(msg)
        res = self.g.solve(left, right)
        states, _ = self.g.to_constant_tgrid(**res)
        
        #TODO: figure out how to make `states` into a message
        self.pc_publisher.publish(states)
        self.cp_publisher.publish(True)

        # we don't need this anymore and the expression graph isn't exactly small, so let's free stuff
        self.destroy_node()




def main(args=None):
    rclpy.init(args=args)
    global_path_node = GlobalPath()
    rclpy.spin(global_path_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
