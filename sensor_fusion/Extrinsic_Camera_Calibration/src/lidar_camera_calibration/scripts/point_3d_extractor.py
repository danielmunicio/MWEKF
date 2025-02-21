import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import os
import ros2_numpy

class Point3DExtractor(Node):
    def __init__(self, velodyne, now):
        super().__init__('point_3d_extractor')
        self.velodyne = velodyne
        self.now = now
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        self.subscription  # prevent unused variable warning

    def clicked_point_callback(self, msg):
        # Extract and print the x, y, z coordinates from the PointStamped message
        point = (msg.point.x, msg.point.y, msg.point.z)
        print(f'Received clicked point: {point}')

    def extract_points_3D(self):
        # Log PID
        print(f'3D Picker PID: [{os.getpid()}]')
        # Extract points data
        points = ros2_numpy.point_cloud2.pointcloud2_to_array(self.velodyne)
        
        points = np.array([(p['x'], p['y'], p['z'], p['intensity']) for p in points.flatten()], dtype=np.float32)
        points = points[~np.isnan(points).any(axis=1)][:]
        if points.shape[0] > 5:
            print(f'PCL points available: {points.shape[0]}')
        else:
            print('Very few PCL points available in range')
            return

        print('Now listening for clicked points in RViz...')

        # Keep the node running to listen for clicked points
        rclpy.spin(self)

    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()