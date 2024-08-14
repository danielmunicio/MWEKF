from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from std_msgs.msg import Float64
from feb_msgs.msg import Cones
from sensor_msgs.msg import Image
import rclpy
import os, glob
import time
import numpy as np
from imutils import paths
import numpy as np
import imutils
import cv2
from camera_only import cone_distance as cd
from .NotCvBridge import imgmsg_to_cv2
import sys
sys.path.append('.')

class ConeNode(Node):
    def __init__(self):
        super().__init__('cone_node')

        # subscribe to images from camera
        self.camera_sub = self.create_subscription(
            Image,
            '/sensors/camera/image_color',
            self.camera_callback,
            1
        )

        self.cone_pub = self.create_publisher(Cones,'/perception_cones',1)
        self.model = YOLO('bestest.pt')
        self.model.fuse()

    def camera_callback(self, msg: Image):
        # Convert Image msg to CV2 image
        img = imgmsg_to_cv2(msg, 'passthrough')
        
                # Convert ROS Image message to OpenCV image
        cv_image = imgmsg_to_cv2(msg, "bgr8")
        
        # Display the image
        #cv2.imshow("Image Window", cv_image)
        #cv2.waitKey(1)  # Refresh the window

        start = time.perf_counter()

        cones_msg = Cones()
        cones_msg.header.stamp = self.get_clock().now().to_msg()
        cone = cd.find_distances(img, self.model)
        classes = [row[0] for row in cone]
        distances = [row[1] for row in cone]
        angles = [row[2] for row in cone]
        print(classes)
        print(distances)
        print(angles)
        cones_msg.r = distances
        cones_msg.theta = angles
        cones_msg.color = classes

        end = time.perf_counter()

        total_time = end - start


        self.cone_pub.publish(cones_msg)

        self.get_logger().info(f"Time taken: {'%.4f' % total_time}")
def main():
    rclpy.init(args=None)
    cone_node = ConeNode()
    rclpy.spin(cone_node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

