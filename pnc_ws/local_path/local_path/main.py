import rclpy
from rclpy.node import Node
import numpy as np
from feb_msgs.msg import State
from feb_msgs.msg import FebPath
from feb_msgs.msg import Map
from std_msgs.msg import Bool
from .local_opt_settings import LocalOptSettings as settings
from .local_opt_compiled import CompiledLocalOpt
from .ConeOrdering import ConeOrdering

class LocalPath(Node):
    def __init__(self):
        super().__init__("local_path")

        #Publishers
        self.pc_publisher = self.create_publisher(FebPath, '/path/local', 1)

        #Subscribers
        self.pc_subscriber = self.create_subscription(Map, '/slam/map/local', self.listener_cb, 1)
        self.state_subscriber = self.create_subscription(State, '/slam/state', self.state_callback, 1)

        self.g = CompiledLocalOpt(**settings)
        self.state = [0,0,0,0]
        
    def listener_cb(self, msg: Map):
        left, right = ConeOrdering(self.msg, self.state)
        res = self.g.solve(left, right, self.state)
        states, _ = self.g.to_constant_tgrid(0.2, **res)
        
        msg = FebPath()
        msg.x = states[:, 0].flatten().tolist()
        msg.y = states[:, 1].flatten().tolist()
        msg.psi = states[:, 2].flatten().tolist()
        msg.v = states[:, 3].flatten().tolist()
        
        self.pc_publisher.publish(msg)

    def state_callback(self, msg: State):
        self.state = list(msg.carstate)


def main(args=None):
    rclpy.init(args=args)
    local_path_node = LocalPath()
    rclpy.spin(local_path_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()