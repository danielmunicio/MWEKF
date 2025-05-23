import os
import sys
# External modules
import cv2
import numpy as np

# ROS 2 modules
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from transforms3d.euler import mat2euler
import ros2_numpy
import image_geometry
from .file_operations import FileOperations
from .model_operations import ModelOperations
from feb_msgs.msg import Cones, ConesCartesian
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Pose, Point, Point32
import tf2_ros
import tf2_geometry_msgs

# Global variables
FIRST_TIME = True
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_camera_calibration'
UTILITIES_PATH = '/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities'
# UTILITIES_PATH = os.path.join(PKG_PATH, 'utilities')

class DepthAnythingCameraOnly(Node):
    def __init__(self, dual_camera=True):
        super().__init__('sensor_fusion_node')

        logitech_camera_topic = '/sensors/camera/image_raw'

        self.image_sub = self.create_subscription(Image, logitech_camera_topic, self.realsense_callback, qos_profile_sensor_data)
        self.cones_pub = self.create_publisher(ConesCartesian, '/cones/camera_only', 1)
        self.perception_visual_pub = self.create_publisher(PointCloud, '/perception_cones_viz', 1)

        self.camera_info_msg = None
        self.image_msg = None

        self.model_operator = ModelOperations(UTILITIES_PATH)

    def camera_callback(self, msg):
        self.image_msg = msg
        self.depth_image = self.get_depth(self.image_msg)
        self.process()


    def setup(self):
        self.get_logger().info('Setting up camera model')
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = 'camera_frame'
        camera_info_msg.distortion_model = 'plumb_bob'
        h, w, d, k, p = FileOperations.get_intrinsic_parameters(UTILITIES_PATH, self.realsense)
        camera_info_msg.height = int(h)
        camera_info_msg.width = int(w)
        camera_info_msg.d = d
        camera_info_msg.k = k
        camera_info_msg.p = p
        CAMERA_MODEL.fromCameraInfo(camera_info_msg)
        self.R, self.T = FileOperations.get_extrinsic_parameters(UTILITIES_PATH)

    def process(self):
        h, w, _, camera_matrix, _ = FileOperations.get_intrinsic_parameters(UTILITIES_PATH, realsenseCamera=True)
        h, w = 480, 640
        camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
        inv_camera_matrix = np.linalg.inv(camera_matrix)
        segmentation_outputs, classes, conf = self.model_operator.predict(self.realsense_image_msg, CV_BRIDGE)

        x_coordinates = []
        y_coordinates = []
        z_coordinates = []
        included_classes = []
        cones_msg = ConesCartesian()

        for idx, segmentation_output in enumerate(segmentation_outputs):
            if conf[idx] > 0.7:
                mask = np.zeros((h, w), dtype=np.uint8)
                if len(segmentation_output) != 0:
                    segmentation_output = np.array([segmentation_output]).reshape((-1, 1, 2))

                    cv2.fillPoly(mask, [segmentation_output], 1)
                    in_segmentation = np.where(mask == 1)
                    depths = depth_image[in_segmentation]
                    pixels = np.vstack((in_segmentation[1], in_segmentation[0], np.ones_like(in_segmentation[1])))
                    camera_coords = (inv_camera_matrix @ pixels) * depths
                    cone_position = np.median(camera_coords, axis=1)
                    print('-------------------------------------')
                    print("CONE COORDINATES: ", cone_position)
                    print('-------------------------------------')
                    cones_msg.x.append(cone_position[0])
                    cones_msg.y.append(cone_position[1])
                    # Z attribute not part of cones message 
                    #cones_msg.z.append(cone_position[2])

        classesToActual = {0: 0, 1: 1, 2: 7, 3: 8, 4: 9}
        yolo_class_to_feb_class = {8: 2, 1: 1, 0: 0, 7: 0, 9: 0}
        mask_conf = [idx for idx, con in enumerate(conf) if con > 0.7]
        chosen_classes = []
        cones_msg.color = [yolo_class_to_feb_class[classesToActual[int(cls)]] for cls in included_classes]
        if len(cones_msg.x) != 0:
            self.cones_cartesian_pub.publish(cones_msg)
        return x_coordinates, y_coordinates, chosen_classes


    def get_depth(self, image):
        pass