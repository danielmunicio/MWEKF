import os
import sys
# External modules
import cv2
import numpy as np
from time import perf_counter
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

from .visual_debugging import publish_perception_visual
from all_settings.all_settings import CameraOnlySettings as settings

# Global variables
FIRST_TIME = True
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_camera_calibration'
UTILITIES_PATH = '/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities'

# UTILITIES_PATH = os.path.join(PKG_PATH, 'utilities')

class RealsenseCameraOnly(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        realsense_camera_topic = '/camera/realsense2/color/image_raw'
        realsense_depth_camera_topic = '/camera/realsense2/aligned_depth_to_color/image_raw'

        self.image_sub_realsense = self.create_subscription(Image, realsense_camera_topic, self.realsense_callback, qos_profile_sensor_data)
        self.depth_sub_realsense = self.create_subscription(Image, realsense_depth_camera_topic, self.realsense_depth_callback, qos_profile_sensor_data)
        self.cones_cartesian_pub = self.create_publisher(ConesCartesian, '/realsense/d435/cones', 1)
        self.perception_visual_pub = self.create_publisher(PointCloud, '/perception_cones_viz', 1)

        self.camera_info_msg = None
        self.image_msg = None
        self.realsense_image_msg = None
        self.realsense_depth_msg = None

        self.model_operator = ModelOperations(UTILITIES_PATH)
        self.setup()
        self.bridge = CvBridge
        self.get_logger().info('Successfully Initialized Realsense Camera-Only')

    def realsense_callback(self, msg):
        self.realsense_image_msg = msg
        if self.realsense_depth_msg is not None:
            self.process()

    def realsense_depth_callback(self, msg):
        print('recieved depth')
        self.realsense_depth_msg = msg
        start = perf_counter()
        self.process()
        end = perf_counter() 
        print("TOTAL TIME: ", end - start)

    def setup(self):
        self.get_logger().info('Setting up camera model')
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = 'camera_frame'
        camera_info_msg.distortion_model = 'plumb_bob'
        h, w, d, k, p = FileOperations.get_intrinsic_parameters(UTILITIES_PATH, realsenseCamera=True)
        camera_info_msg.height = int(h)
        camera_info_msg.width = int(w)
        camera_info_msg.d = d
        camera_info_msg.k = k
        camera_info_msg.p = p
        CAMERA_MODEL.fromCameraInfo(camera_info_msg)
        self.R, self.T = FileOperations.get_extrinsic_parameters(UTILITIES_PATH)

    def process(self):
        if self.realsense_depth_msg is None or self.realsense_image_msg is None:
            return
        h, w, _, camera_matrix, _ = FileOperations.get_intrinsic_parameters(UTILITIES_PATH, realsenseCamera=True)
        h, w = 480, 640
        camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
        inv_camera_matrix = np.linalg.inv(camera_matrix)
        img = CV_BRIDGE.imgmsg_to_cv2(self.realsense_image_msg, 'bgr8')
        segmentation_outputs, classes, conf = self.model_operator.predict(img)
        depth_image = ros2_numpy.numpify(self.realsense_depth_msg)  # shape (480, 640), dtype=uint16
        depth_image = depth_image.astype(np.float32) / 1000.0  # Convert to meters

        cones_msg = ConesCartesian()

        classesToActual = {0: 0, 1: 1, 2: 7, 3: 8, 4: 9}
        yolo_class_to_feb_class = {8: 2, 1: 1, 0: 0, 7: 0, 9: 0}

        for idx, segmentation_output in enumerate(segmentation_outputs):
            if conf[idx] > settings.yolo_minimum_confidence:
                mask = np.zeros((h, w), dtype=np.uint8)
                if len(segmentation_output) != 0:
                    segmentation_output = np.array([segmentation_output]).reshape((-1, 1, 2))

                    cv2.fillPoly(mask, [segmentation_output], 1)
                    in_segmentation = np.where(mask == 1)
                    depths = depth_image[in_segmentation]
                    pixels = np.vstack((in_segmentation[1], in_segmentation[0], np.ones_like(in_segmentation[1])))
                    camera_coords = (inv_camera_matrix @ pixels) * depths
                    cone_position = np.median(camera_coords, axis=1)
                    cone_pos = np.array([cone_position[2], cone_position[0]])
                    angle = -14 * np.pi / 180
                    R = np.array([[np.cos(angle), -np.sin(angle)],
                                   [np.sin(angle), np.cos(angle)]])
                    cone_pos_global = cone_pos @ R.T
                    cones_msg.x.append(cone_pos_global[0])
                    cones_msg.y.append(cone_pos_global[1])
                    cones_msg.color.append(yolo_class_to_feb_class[classesToActual[classes[idx]]])
                    # Z attribute not part of cones message 
                    #cones_msg.z.append(cone_position[2])

        mask_conf = [idx for idx, con in enumerate(conf) if con > settings.yolo_minimum_confidence]
        chosen_classes = []
        if len(cones_msg.x) != 0:
            self.cones_cartesian_pub.publish(cones_msg)
            print("there")
            if settings.publish_visual:
                print("HERE")
                publish_perception_visual(self, cones_msg.x, cones_msg.y, cones_msg.color)


