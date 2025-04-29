# ROS Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from geometry_msgs.msg import Point32
from feb_msgs.msg import ConesCartesian
import ros2_numpy

# Python Libraries
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from time import perf_counter
from cuml.cluster import DBSCAN as cuDBSCAN
import cupy as cp

# Python Files
from all_settings.all_settings import LiDAROnlySettings as settings
from .Calibration.visual_debugging import publish_filtered_pointcloud, plot_clusters_3d

class LiDARCones(Node):
    def __init__(self):
        super().__init__('lidar_only')
        self.z_values = None
        self.recieved = False
        self.saved = False
        self.lidar_sub = self.create_subscription(PointCloud2, 'rslidar_points', self.lidar_callback, 1)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_pointcloud', 1)

        self.a, self.b, self.c, self.d = settings.ground_plane_coefficients
        self.plane_normal = np.sqrt(self.a**2 + self.b**2 + self.c**2)

    def lidar_callback(self, pointcloud):
        start = perf_counter()
        points_xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['xyz']
        intensities = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['intensity']

        points = np.hstack([points_xyz, intensities])
        points = points[~np.isnan(points).any(axis=1)]
        extract_points = perf_counter()

    def save_pointcloud_to_csv(self, points_xyz, filename='saved_pointcloud.csv'):
        # Save points_xyz (N, 3) to a CSV
        np.savetxt(filename, points_xyz, delimiter=',', header='x,y,z', comments='')
        self.get_logger().info(f'Saved pointcloud to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = LiDARCones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
























def __init__(self):
    super().__init__('lidar_only')
    self.z_values = None
    self.recieved = False
    self.saved = False  # <- Add this line
    self.lidar_sub = self.create_subscription(PointCloud2, 'rslidar_points', self.lidar_callback, 1)
    self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_pointcloud', 1)

    self.a, self.b, self.c, self.d = settings.ground_plane_coefficients
    self.plane_normal = np.sqrt(self.a**2 + self.b**2 + self.c**2)

def lidar_callback(self, pointcloud):
    start = perf_counter()
    points_xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['xyz']
    intensities = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['intensity']

    points = np.hstack([points_xyz, intensities])
    points = points[~np.isnan(points).any(axis=1)]
    extract_points = perf_counter()

    if not self.saved:
        self.save_pointcloud_to_csv(points_xyz)
        self.saved = True