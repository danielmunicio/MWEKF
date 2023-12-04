from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy

from feb_msgs.msg import Bitmasks


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        
        # SUBSCRIBERS
        self.mask_sub = self.create_subscription(
            Bitmasks,
            '/perception/masks',
            self.camera_callback,
            1)
        # Add lidar subscriber
        
        # PUBLISHERS
        # For testing
        self.mask_pub = self.create_publisher(Image, '/perception/bitmasks', 1)



    def camera_callback(self, msg: Bitmasks):
        self.get_logger().info(f"Stamp : {msg.header.stamp}")
        
def main():
    rclpy.init(args=None)
    fusion_node = SensorFusionNode()
    rclpy.spin(fusion_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
