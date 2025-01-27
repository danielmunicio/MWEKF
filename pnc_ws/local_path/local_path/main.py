import rclpy
from rclpy.node import Node
import numpy as np
from feb_msgs.msg import State
from feb_msgs.msg import FebPath
from feb_msgs.msg import Map
from std_msgs.msg import Bool
from all_settings.all_settings import LocalOptSettings as settings
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from .local_opt_compiled import CompiledLocalOpt
from .ConeOrdering import ConeOrdering
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion, Point
import matplotlib.pyplot as plt
from time import sleep
from .distance_cone_order import distance_cone_order
from .ConeHistory import ConeHistory

class LocalPath(Node):
    def __init__(self):
        super().__init__("local_path")
        self.cone_history = ConeHistory()

        #Publishers
        self.pc_publisher = self.create_publisher(FebPath, '/path/local', 1)
        self.pointcloud_pub = self.create_publisher(PointCloud, '/path/local/viz', 5)
        self.path_pub = self.create_publisher(Path, '/path/local/vizpath', 1)

        #Subscribers
        self.pc_subscriber = self.create_subscription(Map, '/slam/map/local', self.listener_cb, 1)
        self.pc_subscriber = self.create_subscription(Map, '/slam/map/global', self.global_listener_cb, 1)
        self.pc_subscriber = self.create_subscription(Bool, '/path/finished', self.finished_cb, 1)
        self.state_subscriber = self.create_subscription(State, '/slam/state', self.state_callback, 1)
        print("here")
        self.g = CompiledLocalOpt(**settings)
        print("inited local opt")
        # self.g.construct_solver()
        self.g.construct_solver(generate_c=False, compile_c=False, use_c=True)
        print("constructed solver")
        self.state = [0.,0.,0.,0.]
        print("done")
        self.fails = -1
        self.finished = False
    def finished_cb(self, msg: Bool):
        self.finished = True

    def global_listener_cb(self, msg):
        self.destroy_node() # we don't need anymore after we get the global path
        
    def listener_cb(self, msg: Map):
        if self.finished: return
        #lists are reversed
        if len(list(msg.left_cones_x))<=1 or len(list(msg.right_cones_x))<=1:
            return
        
        left, right = ConeOrdering(msg, self.state, self.cone_history)

        # reverse_left = np.vstack(sorted(np.array(reverse_left), key = lambda x: np.linalg.norm(x-self.state[:2])))
        # reverse_right = np.vstack(sorted(np.array(reverse_right), key = lambda x: np.linalg.norm(x-self.state[:2])))

        #Reverse lists

        left = np.array(left)#[::-1]
        right = np.array(right)#[::-1]
        # print("SHAPE:", left.shape, right.shape)
        with open("sim_data.txt", "a") as f:
            print("---------------------------------------------", file = f)
            # print("FROM PNC: ", file =f)
            print(f"state:\t{self.state}", file = f)
            # print(f"left:\t{left}", file=f)
            # print(f"right:\t{right}", file=f)
            # print("--------------------------------------------", file = f)
            # print(file=f)

        try:
            self.res = self.g.solve(left, right, self.state, err_ok=(self.fails>-0.5 and self.fails <=2))
            self.fails=0
        except RuntimeError:
            self.fails += 1
            if 1 <= self.fails <= 2:
                return
        except UnboundLocalError:
            return
        res = self.res
        print("ok after solve try/except")
        states, _ = self.g.to_constant_tgrid(0.02, **res)
        print("ok converting to constant tgrid")
        # states = np.zeros((50, 4))
        path_msg = FebPath()
        print("ok creating FebPath message")
        path_msg.x = states[:, 0].flatten().tolist()
        path_msg.y = states[:, 1].flatten().tolist()
        path_msg.psi = states[:, 2].flatten().tolist()
        path_msg.v = states[:, 3].flatten().tolist()
        print('MAP Message About to Publish')
        # with open("sim_data.txt", "a") as f:
        #     print("---------------------------------------")
        #     print("From PNC: ")
        #     print("local path:", path_msg, file=f)
        #     print("----------------------------------------")
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

        m = Path()
        poses = []
        for x in states:
            poses.append(PoseStamped())
            poses[-1].pose.position.x = x[0]
            poses[-1].pose.position.y = x[1]
            poses[-1].pose.position.z = 0.0
            poses[-1].header.frame_id = "map"
            poses[-1].pose.orientation.w = np.cos(x[2]/2)
            poses[-1].pose.orientation.x = 0.0
            poses[-1].pose.orientation.y = 0.0
            poses[-1].pose.orientation.z = np.sin(x[2]/2)
            # print("APPENDED:", poses[-1])

        m.poses = poses
        m.header.frame_id = "map"

        self.path_pub.publish(m)
        print(m)


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