from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        self.camera_sub = self.create_subscription(
            Image,
            '/color/image_raw',
            self.camera_callback,
            10)
        self.camera_sub
        
        self.model = YOLO('yolov8n.pt')
        self.model.fuse()
        
    
    def camera_callback(self, msg: Image):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)
        

def main():
    rclpy.init(args=None)
    camera_subscriber = YoloNode()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
