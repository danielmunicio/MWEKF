# ROS Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from geometry_msgs.msg import Point32
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
        self.recieved = False
        self.lidar_sub = self.create_subscription(PointCloud2, 'rslidar_points', self.lidar_callback, 1)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_pointcloud', 1)
        self.marker_pub = self.create_publisher(Marker, '/ground_plane_marker', 1)

        self.a, self.b, self.c, self.d = settings.ground_plane_coefficients
        self.plane_normal = np.sqrt(self.a**2 + self.b**2 + self.c**2)

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
        publish_ground_plane_marker(self)
        

    def filter_points_by_plane_and_distance(self, points, threshold=0.05, max_distance=10.0):
        """
        Vectorized filtering of points based on:
        - Distance from a plane
        - Distance from the origin
        """
        # Plane equation: ax + by + cz + d = 0
        xyz = points[:, :3]  # Shape (N, 3)
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
