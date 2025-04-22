import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.pyplot as plt

# Plane coefficients from the equation
a, b, c, d = -0.055, -0.003, 0.998, 0.591
plane_normal = np.sqrt(a**2 + b**2 + c**2)

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

        # Read points with intensity
        points = list(pc2.read_points(
            pointcloud,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        ))

        # Filter points based on plane distance
        filtered_points = self.filter_points_by_plane(points)

        self.get_logger().info(f"Original: {len(points)} pts, Filtered: {len(filtered_points)} pts")

        # Extract z values for histogram
        self.z_values = [p[2] for p in filtered_points]

        # Publish filtered pointcloud
        self.publish_filtered_pointcloud(filtered_points, pointcloud.header)

        # Plot histogram
        #self.plot_histogram(self.z_values)

        #self.recieved = True

    def filter_points_by_plane(self, points, threshold=0.05):
        """
        Filters out points close to the plane described by the equation.
        Points with a distance from the plane below `threshold` are removed.
        """
        filtered_points = []
        for p in points:
            x, y, z = p[0], p[1], p[2]
            # Compute the signed distance from the point to the plane
            distance = abs(a * x + b * y + c * z + d) / plane_normal
            if distance > threshold:  # Keep points that are far from the plane
                filtered_points.append(p)
        return filtered_points

    def plot_histogram(self, z_values):
        if not z_values:
            self.get_logger().info("No valid z values received.")
            return

        plt.figure(figsize=(10, 6))
        plt.hist(z_values, bins=30, color='skyblue', edgecolor='black')
        plt.xlabel("Height (z)")
        plt.ylabel("Number of Points")
        plt.title("Histogram of Point Heights")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def publish_filtered_pointcloud(self, points, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.filtered_pub.publish(cloud_msg)
        self.get_logger().info("Filtered point cloud published to /filtered_pointcloud")


def main(args=None):
    rclpy.init(args=args)
    node = LiDARCones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
