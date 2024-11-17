import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from NotCvBridge import cv2_to_imgmsg
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.image_publisher = self.create_publisher(Image, '/sensors/camera/image_color', 10)
        self.info_publisher = self.create_publisher(CameraInfo, '/sensors/camera/camera_info', 10)
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.bridge = CvBridge()
        
        while True: 
            ret, frame = self.cap.read()
            if ret:
                height, width, _ = frame.shape
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                img_msg = cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_publisher.publish(img_msg)

                # Create and publish CameraInfo message
                camera_info = CameraInfo()
                camera_info.header.stamp = self.get_clock().now().to_msg()
                camera_info.header.frame_id = "camera_frame"
                # Add more CameraInfo details if available
                self.info_publisher.publish(camera_info)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
