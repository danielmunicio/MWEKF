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


# Global variables
FIRST_TIME = True
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_camera_calibration'
UTILITIES_PATH = '/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities'
# UTILITIES_PATH = os.path.join(PKG_PATH, 'utilities')

class SensorFusion(Node):
    def __init__(self, dual_camera=True):
        super().__init__('sensor_fusion_node')
        logitech_camera_topic = '/sensors/camera/image_color'
        realsense_camera_topic = '/camera/camera/color/image_raw'
        lidar_points_topic = 'rslidar_points'

        self.lidar_sub = self.create_subscription(PointCloud2, lidar_points_topic, self.velodyne_callback, qos_profile_sensor_data)
        self.image_sub_logitech = self.create_subscription(Image, logitech_camera_topic, self.logitech_callback, qos_profile_sensor_data)

        if dual_camera:
            self.image_sub_realsense = self.create_subscription(Image, realsense_camera_topic, self.realsense_callback, qos_profile_sensor_data)

        self.cones_cartesian_pub = self.create_publisher(ConesCartesian, '/perception_cones_cartesian', 1)
        self.perception_pub = self.create_publisher(Cones, '/perception_cones', 1)
        self.perception_visual_pub = self.create_publisher(PointCloud, '/perception_cones_viz', 1)

        self.camera_info_msg = None
        self.image_msg = None
        self.realsense_image_msg = None
        self.logitech_image_msg = None
        self.velodyne_msg = None
        self.combined = dual_camera
        self.model_operator = ModelOperations(UTILITIES_PATH)

    def camera_info_callback(self, msg):
        if not FIRST_TIME:
            return
        self.camera_info_msg = msg
        self.setup()

    def realsense_callback(self, msg):
        self.realsense_image_msg = msg
        self.process()

    def logitech_callback(self, msg):
        self.logitech_image_msg = msg
        self.process() 

    def velodyne_callback(self, msg):
        self.velodyne_msg = msg
        self.process()


    def setup(self):
        global TF_BUFFER, TF_LISTENER, CAMERA_MODEL
        FIRST_TIME = False

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
        if self.combined:
            self.project_both_point_clouds
        else: 
            self.project_point_cloud()



    def get_dist_angle_classes(self, is_realsense=False):
        if self.velodyne_msg is None or (is_realsense and self.realsense_image_msg) is None or (not is_realsense and self.logitech_image_msg) is None:
            return
        # Convert camera into LiDAR coordinate system
        R, T = FileOperations.get_extrinsic_parameters(UTILITIES_PATH, is_realsense)
        RT = np.hstack((R, T.T))
        h, w, _, camera_matrix, _ = FileOperations.get_intrinsic_parameters(UTILITIES_PATH, is_realsense)
        h, w = 480, 640
        camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
        P = camera_matrix @ RT

        points3D = ros2_numpy.point_cloud2.point_cloud2_to_array(self.velodyne_msg)['xyz']
        points3D = np.array(points3D, dtype=np.float64)
        points3D_homogeneous = np.hstack((points3D[:, :3], np.ones((points3D.shape[0], 1))))
        points2D_homogeneous = P @ points3D_homogeneous.T
        points3D_homogeneous = np.hstack((points3D[:, :3], np.ones((points3D.shape[0], 1))))
        points2D_homogeneous = P @ points3D_homogeneous.T
        points2D = points2D_homogeneous[:2, :] / points2D_homogeneous[2, :]
        points2D = points2D.T
        inrange = np.where((points2D[:, 0] >= 0) &
                        (points2D[:, 1] >= 0) &
                        (points2D[:, 0] < h - 1) &
                        (points2D[:, 1] < w - 1))
        points2D = points2D[inrange[0]].round().astype('int')
        points3D = points3D[inrange[0]]
        if is_realsense:
            segmentation_outputs, classes, conf = self.model_operator.predict(self.realsense_image_msg, CV_BRIDGE)
        else:
            segmentation_outputs, classes, conf = self.model_operator.predict(self.logitech_image_msg, CV_BRIDGE)

        x_coordinates = []
        y_coordinates = []
        median_distances = []
        center_points = []
        angles = []
        included_classes = []
        for idx, segmentation_output in enumerate(segmentation_outputs):
            if conf[idx] > 0.7:
                mask = np.zeros((h, w), dtype=np.uint8)
                if len(segmentation_output) != 0:
                    vertices = np.array(segmentation_output, np.int32)
                    centroid = np.average(vertices, axis=0)
                    vertices = np.array([centroid + 0.9 * (v-centroid) for v in vertices], np.int32)
                    cv2.fillPoly(mask, [vertices], 1)
                    x_coords = points2D[:, 0]
                    y_coords = points2D[:, 1]

                    in_polygon = mask[y_coords, x_coords] == 1 
                    points_in_polygon = points3D[in_polygon]
                    x_in_polygon = points_in_polygon[:, 0]
                    y_in_polygon = points_in_polygon[:, 1]

                    if len(points_in_polygon) > 0:
                        median_x = np.median(x_in_polygon)
                        median_y = np.median(y_in_polygon)
                        included_classes.append(classes[idx])
                        x_coordinates.append(median_x)
                        y_coordinates.append(median_y)

        #orange = 0, yellow = 1, blue = 2 in feb system
        classesToActual = {0: 0, 1: 1, 2: 7, 3: 8, 4: 9}
        yolo_class_to_feb_class = {8: 2, 1: 1, 0: 0, 7: 0, 9: 0}
        mask_conf = [idx for idx, con in enumerate(conf) if con > 0.7]
        chosen_classes = []
        chosen_classes = [yolo_class_to_feb_class[classesToActual[int(included_classes[i])]] for i in range(len(included_classes))]
        return x_coordinates, y_coordinates, chosen_classes

    def project_both_point_clouds(self):
        x_coords_realsense, y_coords_realsense, classes_realsense = self.get_dist_angle_classes(self.velodyne_msg, self.realsense_image_msg, self.model_operator, True)
        x_coords_logitech, y_coords_logitech, classes_logitech = self.get_dist_angle_classes(self.velodyne_msg, self.logitech_image_msg, self.model_operator, False)

      
       # Combine Cartesian coordinates and colors from both cameras
        x_combined = np.concatenate([x_coords_realsense, x_coords_logitech])
        y_combined = np.concatenate([y_coords_realsense, y_coords_logitech])
        colors_combined = np.concatenate([classes_realsense, classes_logitech])

        # Create a 2D array of points with colors
        points = np.column_stack((x_combined, y_combined, colors_combined))
        if len(points) == 0:
            return

        cones_msg = ConesCartesian()
        cones_msg.header.stamp = self.get_clock().now().to_msg()
        cones_msg.x = x_combined
        cones_msg.y = y_combined
        cones_msg.color = colors_combined
        self.perception_pub.publish(cones_msg)

        cones_guess = PointCloud()
        positions = []
        for x, y, color_value in zip(x_combined, y_combined, colors_combined):
            positions.append(Point32())
            positions[-1].x = x
            positions[-1].y = y
            positions[-1].z = 0.0

        cones_guess.points = positions
        cones_guess.header.frame_id = "rslidar"
        cones_guess.header.stamp = self.get_clock().now().to_msg()
        self.perception_visual_pub.publish(cones_guess)

    def project_point_cloud(self):
        if self.logitech_image_msg is None:
            return
        x_coords, y_coords, chosen_classes = self.get_dist_angle_classes(is_realsense=False)

        cones_msg = Cones()
        cones_msg.header.stamp = self.get_clock().now().to_msg()

        cones_guess = PointCloud()
        cones_cartesian = ConesCartesian()
        positions = []
        for x, y, color_value in zip(x_coords, y_coords, chosen_classes):
            positions.append(Point32())
            positions[-1].x = x
            positions[-1].y = y
            positions[-1].z = 0.0

            cones_cartesian.x.append(x)
            cones_cartesian.y.append(y)
            cones_cartesian.color.append(color_value)
        cones_guess.points = positions
        cones_guess.header.frame_id= "map"
        cones_guess.header.stamp = self.get_clock().now().to_msg()
        cones_cartesian.header.stamp = self.get_clock().now().to_msg()

        self.perception_visual_pub.publish(cones_guess)
        self.cones_cartesian_pub.publish(cones_cartesian)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion(dual_camera=False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

