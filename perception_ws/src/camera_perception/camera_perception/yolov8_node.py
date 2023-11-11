from ultralytics import YOLO
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
import os
import time

from feb_msgs.msg import Bitmasks, MatrixInt

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
        self.img_pub = self.create_publisher(Image, '/perception/mask_debug', 1)
        self.masks_publisher = self.create_publisher(Bitmasks, 'perception/masks', 1)
        
        
        # GLOBAL VARIABLES
        current_path = os.path.dirname(__file__)
        root = current_path.partition('perception_ws')[0]
        # Find the absolute path to the model
        filename = os.path.join(root, 'perception_ws/src/camera_perception/best.pt')
        self.model = YOLO(filename)
        self.model.fuse()
        self.bridge = CvBridge()        
        
    
    def camera_callback(self, msg: Image):
        # Convert message to CV2 image
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        # Do segmentation
        start = time.perf_counter()
        results = self.model(img, verbose=False)
        end = time.perf_counter()
        seg_time = end - start
        
        # Extract bitmasks (THIS IS SLOW!!)
        start = time.perf_counter()
        mask_list = []
        for mask in results[0].masks:
            # Convert the mask to numpy 2D array
            matrix = mask.data.numpy()[0]
            # Create matrix message
            matrix_msg = MatrixInt()
            matrix_msg.rows, matrix_msg.cols = matrix.shape
            matrix_msg.entries = matrix.flatten().astype(int).tolist() # Very slow, needs optimizing
            # Append to list of masks
            mask_list.append(matrix_msg)
        
        # Extract colors
        colors = list(results[0].names.values())
        color_list = []
        for c in colors:
            color_string = String()
            color_string.data = c
            color_list.append(color_string)
        
        
        # Create bitmask list message
        mask_list_msg = Bitmasks()
        mask_list_msg.bitmask_list = mask_list
        mask_list_msg.colors = color_list
        mask_list_msg.header.frame_id = 'camera_color_optical_frame'
        mask_list_msg.header.stamp = self.get_clock().now().to_msg()
        self.masks_publisher.publish(mask_list_msg)
        end = time.perf_counter()
        post_process_time = end - start
        
        # Publish debugging image
        img_msg = self.bridge.cv2_to_imgmsg(results[0].plot())
        self.img_pub.publish(img_msg)
        
        self.get_logger().info(f"Segmentation time: {'%.4f' % seg_time} s.\tPostprocessing time: {'%.4f' % post_process_time} s.")

        

def main():
    rclpy.init(args=None)
    yolo_node = YoloNode()
    rclpy.spin(yolo_node)
    rclpy.shutdown()
