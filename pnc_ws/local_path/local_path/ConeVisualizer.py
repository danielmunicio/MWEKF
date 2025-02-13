from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import numpy as np
from rclpy.node import Node

class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')
        self.pointcloud_pub = self.create_publisher(PointCloud, 'path/local/viz', 5)

    def publish_cones_with_colors(self, leftN_points, rightN_points):
        """Publish left and right cones with different colors to RViz"""
        pc_msg = PointCloud()
        pts = []
        
        # Assign color to left cones (red) and right cones (blue)
        for x in leftN_points:
            pt = Point32()
            pt.x = x[0]
            pt.y = x[1]
            pt.z = 0.0
            pts.append(pt)
        
        for x in rightN_points:
            pt = Point32()
            pt.x = x[0]
            pt.y = x[1]
            pt.z = 0.0
            pts.append(pt)

        # Add points to the message
        pc_msg.points = pts
        
        # Setting frame_id to "map" for RViz
        pc_msg.header.frame_id = "map"
        pc_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the PointCloud message to RViz
        self.pointcloud_pub.publish(pc_msg)
