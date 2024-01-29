from ultralytics import YOLO
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
import os, glob
import time
import numpy as np

from feb_msgs.msg import Bitmasks

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
        self.masks_publisher = self.create_publisher(Bitmasks, '/perception/camera/bitmasks', 1)
        
        
        # Find the name and absolute path to the model
        current_path = os.path.dirname(__file__)
        path_to_repo = current_path.partition('feb-system-integration')[0]
        model_dir = os.path.join(path_to_repo, 'feb-system-integration/perception_ws/src/camera_perception')
        model_files = [file for file in os.listdir(model_dir) if os.path.splitext(file)[1] == '.pt']
        try:
            model_path = os.path.join(model_dir, model_files[0])
            if len(model_files) > 1:
                self.get_logger().warn(f"Multiple YOLO models found in '{model_dir}', currently using '{model_files[0]}'")
        except IndexError:
            self.get_logger().error(f"No YOLO model with .pt extension found in '{model_dir}', node shut down")
            raise SystemExit
        
        # GLOBAL VARIABLES
        self.model = YOLO(model_path)
        self.model.fuse()
        self.bridge = CvBridge()
        
    
    def camera_callback(self, msg: Image):
        """Runs every time an image messge is received from the camera

        Args:
            msg (Image): Image message from camera
        """
        # Convert message to CV2 image
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        # Do segmentation
        start = time.perf_counter()
        results = self.model(img, verbose=False)
        end = time.perf_counter()
        seg_time = end - start
        # In case nothing is detected
        if not results[0].masks:
            self.get_logger().info('No masks created in this image')
            return
        # Extract bitmasks
        start = time.perf_counter()
        mask_list = []
        for mask in results[0].masks:
            # Convert bitmask to image message
            mask_img = mask.data.numpy()[0].astype(np.uint8)
            mask_msg = self.bridge.cv2_to_imgmsg(mask_img, encoding="mono8")
            
            # Append bitmask to list
            mask_list.append(mask_msg)
            
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


if __name__ == '__main__':
    main()