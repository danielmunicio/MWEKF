from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import rclpy

from feb_msgs.msg import Bitmasks


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        
        # SUBSCRIBERS
        self.mask_sub = self.create_subscription(
            Bitmasks,
            '/perception/camera/bitmasks',
            self.camera_callback,
            1)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/perception/lidar/filtered_cloud',
            self.lidar_callback,
            1)
                
        # PUBLISHERS
        # For testing
        self.mask_pub = self.create_publisher(Image, '/debug/masks', 1)

        # Global variables
        self.current_pc = PointCloud2


    def camera_callback(self, msg: Bitmasks) -> None:
        cam_nano = msg.header.stamp.nanosec
        lidar_nano = self.current_pc.header.stamp.nanosec
        
        diff = (cam_nano - lidar_nano) * 1e-9
        if diff < 0:
            diff += 1
        
        #self.get_logger().info(f'Time difference: {diff}')
        
    
    def lidar_callback(self, msg: PointCloud2) -> None:
        self.current_pc = msg
        
def main():
    rclpy.init(args=None)
    fusion_node = SensorFusionNode()
    rclpy.spin(fusion_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
