import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.pyplot as plt 
import open3d as o3d

class LiDARCones(Node):
    def __init__(self):
        super().__init__('lidar_only')
        self.z_values = None
        self.recieved = False
        self.lidar_sub = self.create_subscription(PointCloud2, 'rslidar_points', self.lidar_callback, 1)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_pointcloud', 1)

    def lidar_callback(self, pointcloud):
        if self.recieved: 
            return

        pc = list(pc2.read_points(pointcloud, field_names=["x", "y", "z"], skip_nans=True))
        # Convert list of tuples to regular float64 ndarray
        points_np = np.array([ [p[0], p[1], p[2]] for p in pc ], dtype=np.float64)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)
        o3d.io.write_point_cloud("pointcloud.pcd", pcd)

        self.recieved = True


def main(args=None):
    rclpy.init(args=args)
    node = LiDARCones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 