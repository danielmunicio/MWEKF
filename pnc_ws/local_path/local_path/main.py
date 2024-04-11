import rclpy
from rclpy.node import Node
import numpy as np
from feb_msgs.msg import State
from feb_msgs.msg import FebPath
from feb_msgs.msg import Map
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from .local_opt_settings import LocalOptSettings as settings
from .local_opt_compiled import CompiledLocalOpt
from .ConeOrdering import ConeOrdering
import matplotlib.pyplot as plt

class LocalPath(Node):
    def __init__(self):
        super().__init__("local_path")

        #Publishers
        self.pc_publisher = self.create_publisher(FebPath, '/path/local', 1)
        self.pointcloud_pub = self.create_publisher(PointCloud, 'path/local/viz', 5)

        #Subscribers
        self.pc_subscriber = self.create_subscription(Map, '/slam/map/local', self.listener_cb, 1)
        self.state_subscriber = self.create_subscription(State, '/slam/state', self.state_callback, 1)

        self.g = CompiledLocalOpt(**settings)
        self.g.construct_solver()
        self.state = [0.,0.,0.,0.]
        
    def listener_cb(self, msg: Map):
        print('MAP Message Received')
        left, right = ConeOrdering(msg, self.state)
        with open('out.txt', 'a') as f:
            print(left, file=f)
        res = self.g.solve(left, right, self.state)
        with open('out.txt', 'a') as f:
            print(res, file=f)
        states, _ = self.g.to_constant_tgrid(0.2, **res)
        with open('out.txt', 'a') as f:
            print(states, file=f)
            print('\n\n\n\n\n')
        # states = np.zeros((50, 4))
        path_msg = FebPath()
        path_msg.x = states[:, 0].flatten().tolist()
        path_msg.y = states[:, 1].flatten().tolist()
        path_msg.psi = states[:, 2].flatten().tolist()
        path_msg.v = states[:, 3].flatten().tolist()
        print('MAP Message About to Publish')
        self.pc_publisher.publish(path_msg)
        pc_msg = PointCloud()
        pts = []
        for x in states:
            pts.append(Point32())
            pts[-1].x = x[0]
            pts[-1].y = x[1]
            pts[-1].z = 0.0
        pc_msg.points = pts
        pc_msg.header.frame_id = "base_footprint"
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pointcloud_pub.publish(pc_msg)

        with open('out.txt', 'a') as f:
            print('finished publishing!', file=f)

    def state_callback(self, msg: State):
        with open('out.txt', 'a') as f:
            print(f"state received: {list(msg.carstate)}", file=f)
        self.state = list(msg.carstate)


def main(args=None):
    rclpy.init(args=args)
    local_path_node = LocalPath()
    rclpy.spin(local_path_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()