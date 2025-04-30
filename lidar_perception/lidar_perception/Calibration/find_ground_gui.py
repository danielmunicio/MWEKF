# ROS Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from feb_msgs.msg import ConesCartesian
import ros2_numpy
from visualization_msgs.msg import Marker

# Python Libraries
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from time import perf_counter
from cuml.cluster import DBSCAN as cuDBSCAN
import cupy as cp

# Python Files
from all_settings.all_settings import LiDAROnlySettings as settings
from .visual_debugging import publish_filtered_pointcloud, plot_clusters_3d, publish_raw_pointcloud, publish_ground_plane_marker

class LiDARCones(Node):
    def __init__(self):
        super().__init__('lidar_only')
        self.z_values = None

        self.lidar_sub = self.create_subscription(PointCloud2, 'rslidar_points', self.lidar_callback, 1)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_pointcloud', 1)
        self.marker_pub = self.create_publisher(Marker, '/ground_plane_marker', 1)

        # Declare parameters for plane coefficients
        self.declare_parameter('plane.a', settings.ground_plane_coefficients[0])
        self.declare_parameter('plane.b', settings.ground_plane_coefficients[1])
        self.declare_parameter('plane.c', settings.ground_plane_coefficients[2])
        self.declare_parameter('plane.d', settings.ground_plane_coefficients[3])

        # Initialize plane coefficients
        self.update_plane_coefficients()

        # Timer to check for coefficient changes every 0.5s
        self.create_timer(0.01, self.update_plane_coefficients)

    def update_plane_coefficients(self):
        self.a = self.get_parameter('plane.a').get_parameter_value().double_value
        self.b = self.get_parameter('plane.b').get_parameter_value().double_value
        self.c = self.get_parameter('plane.c').get_parameter_value().double_value
        self.d = self.get_parameter('plane.d').get_parameter_value().double_value
        self.plane_normal = np.sqrt(self.a ** 2 + self.b ** 2 + self.c ** 2)

        self.get_logger().info(f'Updated plane coefficients: a={self.a:.3f}, b={self.b:.3f}, c={self.c:.3f}, d={self.d:.3f}')
        publish_ground_plane_marker(self)

    def lidar_callback(self, pointcloud):
        # Read points with intensity
        points_xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['xyz']
        intensities = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['intensity']

        points = np.hstack([points_xyz, intensities])
        points = points[~np.isnan(points).any(axis=1)]

        filtered_points = self.filter_points_by_plane_and_distance(
            points,
            threshold=settings.ground_filter_threshold,
            max_distance=settings.max_distance
        )

        #publish_raw_pointcloud(self, filtered_points, pointcloud.header)

    def filter_points_by_plane_and_distance(self, points, threshold=0.05, max_distance=10.0):
        xyz = points[:, :3]
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        distance_to_plane = np.abs(self.a * x + self.b * y + self.c * z + self.d) / self.plane_normal
        distance_from_origin = np.sqrt(x**2 + y**2 + z**2)

        #mask = (distance_to_plane > threshold) & (distance_from_origin <= max_distance)
        mask = distance_from_origin <= max_distance
        return points[mask]


def main(args=None):
    rclpy.init(args=args)
    node = LiDARCones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
