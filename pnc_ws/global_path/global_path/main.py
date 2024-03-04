import rclpy
from rclpy.node import Node
import numpy as np
from feb_msgs.msg import State
from feb_msgs.msg import FebPath
from feb_msgs.msg import Cones
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
        self.pc_subscriber = self.create_subscription(Cones, '/slam/matched/global', self.listener_cb, 10)

        self.g = CompiledGlobalOpt(**settings)

    def listener_cb(self, msg: Cones):
        left, right = ConeOrdering(msg)
        res = self.g.solve(left, right)
        states, _ = self.g.to_constant_tgrid(**res)
        
        msg = FebPath()
        msg.state_path = [State() for _ in states]
        for i, state in enumerate(msg.state_path):
            state.carstate = states[i].tolist()
            
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