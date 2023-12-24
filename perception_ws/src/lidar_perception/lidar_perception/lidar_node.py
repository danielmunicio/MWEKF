from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import rclpy
import time


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_node')
        
        # SUBSCRIBERS
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.lidar_callback,
            10)
        
        # PUBLISHERS
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/perception/lidar/filtered_cloud',
            1)
        
        # Global varibles
        self.mount_height = 0.6
        self.upper_threshold = 0.3
        self.max_z = self.upper_threshold - self.mount_height
        self.max_range = 15

    def lidar_callback(self, msg: PointCloud2) -> None:
        # Convert PointCloud2 message to a numpy array
        start = time.perf_counter()
        pc_data = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, len(msg.fields)))
        end = time.perf_counter()
        conv_time = end - start
        
        # Get the index of the 'z' field
        z_index = next(i for i, field in enumerate(msg.fields) if field.name == 'z')

        # Set the 'z' values to NaN for points with range greater than the critical value
        start = time.perf_counter()
        range_values = np.sqrt(pc_data[:, :, 0] ** 2 + pc_data[:, :, 1] ** 2 + pc_data[:, :, 2] ** 2)
        mask = (range_values <= self.max_range) & (pc_data[:, :, z_index] <= self.max_z)
        pc_data[~mask, z_index] = np.nan

        # Create a new PointCloud2 message
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = msg.height
        filtered_msg.width = msg.width
        filtered_msg.fields = msg.fields
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = msg.row_step
        filtered_msg.is_dense = msg.is_dense

        # Set the modified array as the data field of the new PointCloud2 message
        filtered_msg.data = pc_data.tobytes(order='C') # Very slow!!!
        end = time.perf_counter()
        filter_time = end - start

        # Publish the modified PointCloud2 message
        self.filtered_pub.publish(filtered_msg)
        self.get_logger().info(f'Conversion time: {"%.3f" % conv_time}\tFiltering time: {"%.3f" % filter_time}')
        

def main(args=None):
    rclpy.init(args=args)
        
    node = LidarNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
