from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
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
        self.crit_value = self.upper_threshold - self.mount_height

    def lidar_callback(self, msg: PointCloud2) -> None:
        start = time.perf_counter()
        cloud_iterable = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        end = time.perf_counter()
        conv_time = end - start
        
        start = time.perf_counter()
        z_filtered_cloud = [point for point in cloud_iterable if point[2] < self.crit_value]
        end = time.perf_counter()
        filter_time = end - start
        
        print(f'Conversion time: {"%.3f" % conv_time}\nFiltering time: {"%.3f" % filter_time}')
        
        header = Header()
        header.frame_id = 'rslidar'
        header.stamp = self.get_clock().now().to_msg()
        new_msg = pc2.create_cloud_xyz32(header, z_filtered_cloud)
        
        self.filtered_pub.publish(new_msg)
        self.get_logger().info('Published!')
        

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
