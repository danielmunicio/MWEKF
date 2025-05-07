import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from image_geometry import PinholeCameraModel

class DepthAligner(Node):
    def __init__(self):
        super().__init__('depth_aligner')

        self.bridge = CvBridge()

        self.color_info = None
        self.depth_info = None
        self.color_image = None
        self.depth_image = None

        self.color_info_sub = self.create_subscription(CameraInfo, '/camera/realsense2/color/camera_info', self.color_info_callback, 10)
        self.depth_info_sub = self.create_subscription(CameraInfo, '/camera/realsense2/depth/camera_info', self.depth_info_callback, 10)
        self.color_img_sub = self.create_subscription(Image, '/camera/realsense2/color/image_raw', self.color_img_callback, 10)
        self.depth_img_sub = self.create_subscription(Image, '/camera/realsense2/depth/image_rect_raw', self.depth_img_callback, 10)

        self.aligned_pub = self.create_publisher(Image, '/aligned_depth_to_color', 10)

        self.get_logger().info('DepthAligner Node Initialized')

    def color_info_callback(self, msg):
        self.color_info = msg

    def depth_info_callback(self, msg):
        self.depth_info = msg

    def color_img_callback(self, msg):
        self.color_image = msg
        self.try_align()

    def depth_img_callback(self, msg):
        self.depth_image = msg
        self.try_align()

    def try_align(self):
        if self.color_info and self.depth_info and self.color_image and self.depth_image:
            aligned = self.align_depth_to_color()
            if aligned is not None:
                aligned_msg = self.bridge.cv2_to_imgmsg(aligned, encoding='passthrough')
                aligned_msg.header = self.color_image.header
                self.aligned_pub.publish(aligned_msg)

    def align_depth_to_color(self):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='passthrough')
            color_image = self.bridge.imgmsg_to_cv2(self.color_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CVBridge conversion failed: {e}')
            return None

        depth_model = PinholeCameraModel()
        color_model = PinholeCameraModel()
        depth_model.fromCameraInfo(self.depth_info)
        color_model.fromCameraInfo(self.color_info)

        height, width = color_image.shape[:2]
        aligned_depth = np.zeros((height, width), dtype=np.uint16)

        fx_d, fy_d, cx_d, cy_d = depth_model.fx(), depth_model.fy(), depth_model.cx(), depth_model.cy()
        fx_c, fy_c, cx_c, cy_c = color_model.fx(), color_model.fy(), color_model.cx(), color_model.cy()

        # Identity extrinsics (assume same origin if you don’t have the transform)
        R = np.eye(3)
        T = np.zeros(3)

        for v in range(depth_image.shape[0]):
            for u in range(depth_image.shape[1]):
                z = depth_image[v, u] * 0.001  # mm to meters
                if z == 0:
                    continue

                x = (u - cx_d) * z / fx_d
                y = (v - cy_d) * z / fy_d
                point_d = np.array([x, y, z])

                point_c = R @ point_d + T
                u_c = int((point_c[0] * fx_c / point_c[2]) + cx_c)
                v_c = int((point_c[1] * fy_c / point_c[2]) + cy_c)

                if 0 <= u_c < width and 0 <= v_c < height:
                    aligned_depth[v_c, u_c] = int(point_c[2] * 1000)

        return aligned_depth

def main(args=None):
    rclpy.init(args=args)
    node = DepthAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
