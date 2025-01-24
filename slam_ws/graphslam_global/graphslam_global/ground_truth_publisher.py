import numpy as np 
import rclpy 
from rclpy.node import Node
from eufs_msgs.msg import CarState, ConeArrayWithCovariance
from feb_msgs.msg import State, Map
from .utility_functions import quat_to_euler

class Ground_Truth_Publisher(Node):
    def __init__(self):

        super().__init__('ground_truth_publisher_node')

        # Ground Truth Subscribers 
        self.cones_sub = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones',
            self.cones_callback,
            1
        )

        self.state_sub = self.create_subscription(
            CarState,
            '/ground_truth/state',
            self.state_callback,
            1
        )

        # PUBLISHERS
        # Publish the current vehicle's state: X, Y, Velo, Theta
        self.state_pub = self.create_publisher(
            State,
            '/slam/state',
            1
        )

        # Publish the current map (GLOBAL_NODE, so this will send the whole map)
        self.global_map_pub = self.create_publisher(
            Map, 
            '/slam/map/global',
            1
        )

        self.local_map_pub = self.create_publisher(
            Map, 
            '/slam/map/local',
            1
        )
        self.blue_cones = np.zeros((0, 2))
        self.yellow_cones = np.zeros((0, 2))
    def cones_callback(self, cones: ConeArrayWithCovariance) -> None:

        for cone in cones.blue_cones: 
            new_cone = True
            for mapped_cone in self.blue_cones:
                if np.linalg.norm([(cone.point.x, cone.point.y), (mapped_cone[0], mapped_cone[1])]) < 0.1:
                    new_cone = False
                    break
            if new_cone:
                self.blue_cones = np.vstack([self.blue_cones, [cone.point.x, cone.point.y]])

        for cone in cones.yellow_cones: 
            new_cone = True
            for mapped_cone in self.yellow_cones:
                if np.linalg.norm([(cone.point.x, cone.point.y), (mapped_cone[0], mapped_cone[1])]) < 0.1:
                    new_cone = False
                    break
            if new_cone:
                self.yellow_cones = np.vstack([self.yellow_cones, [cone.point.x, cone.point.y]])


        cones_map = Map()
        cones_map.left_cones_x = self.blue_cones[0].astype(float).tolist()
        cones_map.left_cones_y = self.blue_cones[1].astype(float).tolist()
        cones_map.right_cones_x = self.yellow_cones[0].astype(float).tolist()
        cones_map.right_cones_y = self.yellow_cones[1].astype(float).tolist()
        self.local_map_pub.publish(cones_map)

    def state_callback(self, state: CarState) -> None:
        car_state = State()
        car_state.x = state.pose.pose.position.x
        car_state.y = state.pose.pose.position.y
        car_state.heading = quat_to_euler(state.pose.pose.orientation)
        car_state.velocity = np.sqrt(state.twist.twist.linear.x**2 + state.twist.twist.linear.y**2)
        car_state.lap_count = 0
        self.state_pub.publish(car_state)