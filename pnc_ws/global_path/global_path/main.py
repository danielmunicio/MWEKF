import rclpy
from rclpy.node import Node
import numpy as np
from feb_msgs.msg import State
from feb_msgs.msg import FebPath
from feb_msgs.msg import Map
from std_msgs.msg import Bool
from .global_opt_settings import GlobalOptSettings as settings
from .global_opt_compiled import CompiledGlobalOpt
from .ConeOrdering import ConeOrdering

class GlobalPath(Node):
    def __init__(self):
        super().__init__("global_path")

        #Publishers
        self.pc_publisher = self.create_publisher(FebPath, '/path/global', 10)
        self.cp_publisher = self.create_publisher(Bool, '/path/finished', 10)
        
        #Subscribers
        self.pc_subscriber = self.create_subscription(Map, '/slam/matched/global', self.listener_cb, 10)

        self.g = CompiledGlobalOpt(**settings)

    def listener_cb(self, msg: Map):
        left, right = ConeOrdering(msg)
        res = self.g.solve(left, right)
        states, _ = self.g.to_constant_tgrid(**res)
        
        msg = FebPath()
        msg.x = states[:, 0].flatten().tolist()
        msg.y = states[:, 1].flatten().tolist()
        msg.psi = states[:, 2].flatten().tolist()
        msg.v = states[:, 3].flatten().tolist()
        
        self.pc_publisher.publish(msg)

        msg = Bool()
        msg.data = True
        self.cp_publisher.publish(msg)

        # we don't need this anymore and the expression graph isn't exactly small, so let's free stuff
        self.destroy_node()




def main(args=None):
    rclpy.init(args=args)
    global_path_node = GlobalPath()
    rclpy.spin(global_path_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()