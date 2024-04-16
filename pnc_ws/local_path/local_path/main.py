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
        self.pc_subscriber = self.create_subscription(Bool, '/path/finished', self.finished_cb, 1)
        self.state_subscriber = self.create_subscription(State, '/slam/state', self.state_callback, 1)

        self.g = CompiledLocalOpt(**settings)
        self.g.construct_solver()
        self.state = [0.,0.,0.,0.]

        self.finished = False
    def finished_cb(self, msg: Bool):
        self.finished = True
        
    def listener_cb(self, msg: Map):
        if self.finished: return
        print('MAP Message Received')
        #lists are reversed
        if len(list(msg.left_cones_x))<=1 or len(list(msg.right_cones_x))<=1:
            return
        
        reverse_left, reverse_right = ConeOrdering(msg, self.state)

        reverse_left = np.vstack(sorted(np.array(reverse_left), key = lambda x: np.linalg.norm(x-self.state[:2])))
        reverse_right = np.vstack(sorted(np.array(reverse_right), key = lambda x: np.linalg.norm(x-self.state[:2])))

        #Reverse lists
        left = reverse_left#[::-1]
        right = reverse_right#[::-1]
        with open("sim_data.txt", "a") as f:
            print("---------------------------------------------", file = f)
            print("FROM PNC: ", file =f)
            print(f"state:\t{self.state}", file = f)
            print(f"left:\t{left}", file=f)
            print(f"right:\t{right}", file=f)
            print("--------------------------------------------", file = f)
            print(file=f)
        res = self.g.solve((left+right)/2, (left+right)/2, self.state)
        states, _ = self.g.to_constant_tgrid(0.2, **res)
        # states = np.zeros((50, 4))
        path_msg = FebPath()
        path_msg.x = states[:, 0].flatten().tolist()
        path_msg.y = states[:, 1].flatten().tolist()
        path_msg.psi = states[:, 2].flatten().tolist()
        path_msg.v = states[:, 3].flatten().tolist()
        print('MAP Message About to Publish')
        with open("sim_data.txt", "a") as f:
            print("---------------------------------------")
            print("From PNC: ")
            print("local path:", path_msg, file=f)
            print("----------------------------------------")
        self.pc_publisher.publish(path_msg)
        pc_msg = PointCloud()
        pts = []
        
        leftmap, rightmap = (
            np.array([list(msg.left_cones_x), list(msg.left_cones_y)]).T,
            np.array([list(msg.right_cones_x), list(msg.right_cones_y)]).T,
        )
        fwd = np.array([self.state[:2]]) + 0.5*np.array([[np.cos(self.state[2]), np.sin(self.state[2])]])
        print(fwd.shape)
        for x in np.vstack([left, right, states[:, :2], np.array([self.state[:2]]), fwd]):
            pts.append(Point32())
            pts[-1].x = x[0]
            pts[-1].y = x[1]
            pts[-1].z = 0.0
        pc_msg.points = pts
        pc_msg.header.frame_id = "map"
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pointcloud_pub.publish(pc_msg)


    def state_callback(self, carstate: State):
        self.state = [0, 0, 0, 0]
        self.state[0] = carstate.x # x value
        self.state[1] = carstate.y # y value
        ### MPC Assumes the veloccity and heading are flipped
        self.state[2] = carstate.heading # velocity
        self.state[3] = carstate.velocity # heading


def main(args=None):
    rclpy.init(args=args)
    local_path_node = LocalPath()
    rclpy.spin(local_path_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()