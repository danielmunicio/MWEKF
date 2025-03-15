import numpy as np 
import rclpy 
from rclpy.node import Node
from eufs_msgs.msg import CarState, ConeArrayWithCovariance
from feb_msgs.msg import State, Map
from .utility_functions import quat_to_euler
from .ground_truth_cone_map import blue_cones_global, yellow_cones_global
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped
from all_settings.all_settings import GraphSLAMSettings as settings

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

        self.cones_viz_pub = self.create_publisher(
            PointCloud, 
            'slam/conemap',
            1
        )

        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/slam/pose',
            1
        )
        self.blue_cones = np.zeros((0, 2))
        self.yellow_cones = np.zeros((0, 2))
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.instant_global_map = settings.instant_global_map
        if self.instant_global_map: 
            # Publisher every 0.5 seconds
            self.timer = self.create_timer(0.5, self.publish_global_map)
    def cones_callback(self, cones: ConeArrayWithCovariance) -> None:
        if self.instant_global_map:
            return
        bloobs = np.array([[i.point.x for i in cones.blue_cones],
                           [i.point.y for i in cones.blue_cones]])
        yellow = np.array([[i.point.x for i in cones.yellow_cones],
                           [i.point.y for i in cones.yellow_cones]])

        rot = lambda theta: np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        # Why does Reid code like this ...
        # This is putting the cones in the global map, the simulator puts them in the local map which is bad 
        pos = np.array([self.x, self.y])[:, np.newaxis]
        bloobs = rot(self.heading)@bloobs + pos
        yellow = rot(self.heading)@yellow + pos

        for cone in bloobs.T:
            new_cone = True
            for mapped_cone in self.blue_cones:
                #print("NORM: ", np.linalg.norm([cone.point.x - mapped_cone[0], cone.point.y - mapped_cone[1]]))
                if np.linalg.norm([cone[0] - mapped_cone[0], cone[1] - mapped_cone[1]]) < 0.1:
                    new_cone = False
                    break
            if new_cone:
                self.blue_cones = np.vstack([self.blue_cones, [cone[0], cone[1]]])
                # print("Blue Cone Map: ", self.blue_cones)

        for cone in yellow.T:
            new_cone = True
            for mapped_cone in self.yellow_cones:
                if np.linalg.norm([cone[0] - mapped_cone[0], cone[1] - mapped_cone[1]]) < 0.1:
                    new_cone = False
                    break
            if new_cone:
                self.yellow_cones = np.vstack([self.yellow_cones, [cone[0], cone[1]]])
                # print("Yellow Cone Map: ", self.yellow_cones)


        cones_map = Map()
        cones_map.left_cones_x = self.blue_cones[:, 0].astype(float).tolist()
        cones_map.left_cones_y = self.blue_cones[:, 1].astype(float).tolist()
        cones_map.right_cones_x = self.yellow_cones[:, 0].astype(float).tolist()
        cones_map.right_cones_y = self.yellow_cones[:, 1].astype(float).tolist()

        self.local_map_pub.publish(cones_map)


        cones_viz = PointCloud()
        cones_viz.header.frame_id = "map"
        cones_viz.header.stamp = self.get_clock().now().to_msg()

        for x, y in zip(cones_map.left_cones_x, cones_map.left_cones_y):
            point = Point32(x=x, y=y, z=0.0)
            cones_viz.points.append(point)

        for x, y in zip(cones_map.right_cones_x, cones_map.right_cones_y):
            point = Point32(x=x, y=y, z=0.0)
            cones_viz.points.append(point)

        self.cones_viz_pub.publish(cones_viz)

    def state_callback(self, state: CarState) -> None:
        self.x = state.pose.pose.position.x
        self.y = state.pose.pose.position.y
        self.heading = quat_to_euler(state.pose.pose.orientation)

        car_state = State()
        car_state.x = self.x
        car_state.y = self.y
        car_state.heading = self.heading
        car_state.velocity = np.sqrt(state.twist.twist.linear.x**2 + state.twist.twist.linear.y**2)
        car_state.lap_count = 0
        self.state_pub.publish(car_state)#

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.
        pose_msg.pose.orientation.w = np.cos(self.heading/2)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = np.sin(self.heading/2)
    
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        self.pose_pub.publish(pose_msg)

    def publish_global_map(self) -> None:
        print("Publishing Global Map")
        cones_map = Map()
        cones_map.left_cones_x = blue_cones_global[:, 0].astype(float).tolist()
        cones_map.left_cones_y = blue_cones_global[:, 1].astype(float).tolist()
        cones_map.right_cones_x = yellow_cones_global[:, 0].astype(float).tolist()
        cones_map.right_cones_y = yellow_cones_global[:, 1].astype(float).tolist()
        self.global_map_pub.publish(cones_map)