from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import struct
import numpy as np
from geometry_msgs.msg import Point32

class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')
        # point cloud publisher
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'cones/viz', 10)

    def publish_cones_with_colors(self, leftN_points, rightN_points):
        all_cones = np.vstack([leftN_points, rightN_points])
        
        # header for message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        
        # Create list to hold point data (x, y, z, rgb)
        points = []
        
        # Assign color values (red for left, blue for right)
        for idx, x in enumerate(all_cones):
            point = [x[0], x[1], 0.0]
            if idx < len(leftN_points):
                color = 0xFF0000  # Red
            else:
                color = 0x0000FF  # Blue
            point.append(struct.unpack('f', struct.pack('I', color))[0])  # Convert RGB to float
            points.append(point)
        
        # Create PointCloud2 message
        pc2 = point_cloud2.create_cloud_xyz32(header, points)
        
        # publish the message
        self.pointcloud_pub.publish(pc2)
        self.get_logger().info('Published cone points with colors to RViz')
