import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from pcl import PointCloud
from pcl import VoxelGrid


class VoxelFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_only')
        self.z_values = None
        self.recieved = False
        self.filtered_sub = self.create_subscription(PointCloud2, '/filtered_pointcloud', 1)
        self.voxel_pub = self.create_publisher()

    def voxel_filter_callback(point_cloud_msg):
        # Convert PointCloud2 message to PCL PointCloud
        pcl_point_cloud = PointCloud()
        pcl_point_cloud.from_message(point_cloud_msg)

        # Create a Voxel Grid filter object
        voxel_grid = VoxelGrid()
        voxel_grid.setInputCloud(pcl_point_cloud)
        
        # Set the voxel grid size (adjust as needed)
        voxel_grid.setLeafSize(0.01, 0.01, 0.01)

        # Filter the point cloud
        filtered_point_cloud = PointCloud()
        voxel_grid.filter(filtered_point_cloud)

        # Publish the filtered point cloud
        filtered_point_cloud_msg = filtered_point_cloud.to_msg()
        filtered_point_cloud_msg.header = point_cloud_msg.header
        filtered_point_cloud_pub.publish(filtered_point_cloud_msg)  