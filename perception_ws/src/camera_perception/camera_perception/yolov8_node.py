from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from feb_msgs.msg import YoloInference, InferenceResult

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # SUBSCRIBERS
        self.camera_sub = self.create_subscription(
            Image,
            '/color/image_raw',
            self.camera_callback,
            10)
        
        # PUBLISHERS
        self.yolo_pub = self.create_publisher(YoloInference, '/perception/yolo_inference', 1)
        self.img_pub = self.create_publisher(Image, '/perception/inference_result', 1)
        
        # GLOBAL VARIABLES
        self.model = YOLO('yolov8m-seg.pt')
        self.model.fuse()
        self.bridge = CvBridge()
        self.inference = YoloInference()
        
        
    
    def camera_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        width, height = msg.width, msg.height
        results = self.model(img)
        
        # Create inference message header
        self.inference.header.frame_id = "camera_color_optical_frame"
        self.inference.header.stamp = self.get_clock().now().to_msg()
        
        """
        for r in results:
            for mask in r.masks:
                # scale for visualizing results
                msk = mask.data.numpy() * 255
                print(msk.shape)
        """
                
        
        img_msg = self.bridge.cv2_to_imgmsg(results[0].plot())
        #print(img_msg.width, img_msg.height)
        self.img_pub.publish(img_msg)
        
        self.yolo_pub.publish(self.inference)
        self.inference.yolo_inference.clear()
        
        
        
        """
        annotated_frame = results[0].plot()
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolo_pub.publish(self.inference)
        self.inference.yolo_inference.clear()
        """
        

def main():
    rclpy.init(args=None)
    yolo_node = YoloNode()
    rclpy.spin(yolo_node)
    rclpy.shutdown()
