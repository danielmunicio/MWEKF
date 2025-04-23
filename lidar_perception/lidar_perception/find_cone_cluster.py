import os
import sys
import threading
import multiprocessing
import ast
import time 
# External modules
import cv2
import numpy as np
import matplotlib.pyplot as plt

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



    def get_dist_angle_classes(self, velodyne, img_msg, model_operator, is_realsense):
        R, T = FileOperations.get_extrinsic_parameters(UTILITIES_PATH, is_realsense)
        RT = np.hstack((R, T.T))
        h, w, _, camera_matrix, _ = FileOperations.get_intrinsic_parameters(UTILITIES_PATH, is_realsense)
        h, w = 480, 640
        camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
        P = camera_matrix @ RT
        intensities = ros2_numpy.point_cloud2.point_cloud2_to_array(velodyne)['intensity']
        max_intensity = np.max(intensities)
        points3D = ros2_numpy.point_cloud2.point_cloud2_to_array(velodyne)['xyz']
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
        intensities = intensities[inrange[0]]
        segmentation_outputs, classes, conf = model_operator.predict(img_msg, CV_BRIDGE)

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
                    #cv2.fillPoly(mask, [np.array(segmentation_output, np.int32)], 1)
                    x_coords = points2D[:, 0]
                    y_coords = points2D[:, 1]
                    in_polygon = mask[y_coords, x_coords] == 1 
                    points_in_polygon = points3D[in_polygon]
                    if len(points_in_polygon) > 0:
                        angle = np.arctan2(np.median(points_in_polygon[:, 0]), np.median(points_in_polygon[:, 1]))
                        if angle < 0:
                            angle = -((np.pi / 2) + angle)
                        else:
                            angle = (np.pi / 2) - angle
                        angles.append(angle)
                        distances = np.linalg.norm(points_in_polygon[:, :2], axis=1)
                        median_distance = np.median(distances) if len(distances) > 0 else float('nan')
                        median_distances.append(median_distance)
                        included_classes.append(classes[idx])
        #orange = 0, yellow = 1, blue = 2 in feb system
        classesToActual = {0: 0, 1: 1, 2: 7, 3: 8, 4: 9}
        yolo_class_to_feb_class = {8: 2, 1: 1, 0: 0, 7: 0, 9: 0}
        mask_conf = [idx for idx, con in enumerate(conf) if con > 0.7]
        chosen_classes = []
        chosen_classes = [yolo_class_to_feb_class[classesToActual[int(included_classes[i])]] for i in range(len(included_classes))]
        return median_distances, angles, chosen_classes

    def project_both_point_clouds(self):
        dists_realsense, angles_realsense_unmodified, classes_realsense = self.get_dist_angle_classes(self.velodyne_msg, self.realsense_image_msg, self.model_operator, True)
        dists_logitech, angles_logitech_unmodified, classes_logitech = self.get_dist_angle_classes(self.velodyne_msg, self.logitech_image_msg, self.model_operator, False)

        angles_logitech = [angle for angle in angles_logitech_unmodified]
        angles_realsense = [angle for angle in angles_realsense_unmodified]
        cones_msg = Cones()
        cones_msg.header.stamp = self.get_clock().now().to_msg()

        print("LOGITECH STUFF")
        print("CONES R: ", dists_logitech)
        print("CONES THETA: ", angles_logitech)
        print("COLOR: ", classes_logitech)
        print("==================================")
        print("REALSENSE STUFF")
        print("CONES R: ", dists_realsense)
        print("CONES THETA: ", angles_realsense)
        print("COLOR: ", classes_realsense)
        print("================================================")
        print("================================================")
        
        cartesian_x_distances_logitech = dists_logitech * np.cos(angles_logitech)
        cartesian_y_distances_logitech = dists_logitech * np.sin(angles_logitech)
        cartesian_x_distances_realsense = dists_realsense * np.cos(angles_realsense)
        cartesian_y_distances_realsense = dists_realsense * np.sin(angles_realsense)

        # Combine Cartesian coordinates and colors from both cameras
        x_combined = np.concatenate([cartesian_x_distances_logitech, cartesian_x_distances_realsense])
        y_combined = np.concatenate([cartesian_y_distances_logitech, cartesian_y_distances_realsense])
        colors_combined = np.concatenate([classes_logitech, classes_realsense])

        # Create a 2D array of points with colors
        points = np.column_stack((x_combined, y_combined, colors_combined))
        if len(points) == 0:
            return
        

        # Convert filtered Cartesian coordinates back to polar
        filtered_r = np.sqrt(filtered_x**2 + filtered_y**2).astype(float)
        filtered_r = [float(r) for r in filtered_r]
        print(type(filtered_r[0]))
        filtered_theta = np.arctan2(filtered_y, filtered_x)

        filtered_theta =[float(theta) for theta in filtered_theta]
        filtered_colors = [int(colors) for colors in filtered_colors]

        cartesian_x_distances = filtered_r * np.cos(filtered_theta)
        cartesian_y_distances = filtered_r * np.sin(filtered_theta)

        cones_msg.r = filtered_r
        cones_msg.theta = filtered_theta
        cones_msg.color = filtered_colors
        self.perception_pub.publish(cones_msg)

        cones_guess = PointCloud()
        positions = []
        for x, y, color_value in zip(cartesian_x_distances, cartesian_y_distances, filtered_colors):
            positions.append(Point32())
            positions[-1].x = x
            positions[-1].y = y
            positions[-1].z = 0.0
        cones_guess.points = positions
        cones_guess.header.frame_id = "map"
        cones_guess.header.stamp = self.get_clock().now().to_msg()
        self.perception_visual_pub.publish(cones_guess)

    def project_point_cloud(self, velodyne, img_msg, image_pub, model_operator, display_projection, perception_pub, realsense):
        median_distances, angles, chosen_classes = self.get_dist_angle_classes(velodyne, img_msg, model_operator, realsense)

        cones_msg = Cones()
        cones_msg.header.stamp = self.get_clock().now().to_msg()

        cartesian_x_distances = median_distances * np.cos(angles)
        cartesian_y_distances = median_distances * np.sin(angles)

        cones_guess = PointCloud()
        cones_cartesian = ConesCartesian()
        positions = []
        for x, y, color_value in zip(cartesian_x_distances, cartesian_y_distances, chosen_classes):
            positions.append(Point32())
            positions[-1].x = x
            positions[-1].y = y
            positions[-1].z = 0.0

            cones_cartesian.x.append(x)
            cones_cartesian.y.append(y)
            cones_cartesian.color.append(color_value)
        cones_guess.points = positions
        cones_guess.header.frame_id= "map"
        cones_guess.header.stamp, cones_cartesian.header.stamp = self.get_clock().now().to_msg()
        self.perception_visual_pub.publish(cones_guess)
        self.cones_cartesian_pub.publish(cones_cartesian)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion(dual_camera=False)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

