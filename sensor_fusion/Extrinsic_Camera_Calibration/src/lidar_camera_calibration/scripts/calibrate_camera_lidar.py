#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Built-in modules
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
from file_operations import FileOperations
from model_operations import ModelOperations
from feb_msgs.msg import Cones
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Pose, Point, Point32


# Global variables
OUSTER_LIDAR = False
PAUSE = False
FIRST_TIME = True
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_camera_calibration'
UTILITIES_PATH = '/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities'
# UTILITIES_PATH = os.path.join(PKG_PATH, 'utilities')

def handle_keyboard():
    global KEY_LOCK, PAUSE
    input('Press [ENTER] to pause and pick points\n')
    with KEY_LOCK: PAUSE = True

def start_keyboard_handler():
    keyboard_t = threading.Thread(target=handle_keyboard)
    keyboard_t.daemon = True
    keyboard_t.start()

def extract_points_2D(img_msg, now, rectify=False):
    # Read image using CV bridge
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
    except CvBridgeError as e: 
        print(e)
        return

    # Rectify image
    if rectify: CAMERA_MODEL.rectifyImage(img, img)
    disp = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)

    # Setup matplotlib GUI
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('Select 2D Image Points - %d' % now)
    ax.set_axis_off()
    ax.imshow(disp)

    # Pick points
    picked, picked_full_list = [], []
    def onclick(event):
        x = event.xdata
        y = event.ydata
        if (x is None) or (y is None): return

        # Display the picked point
        picked.append((x, y))
        picked_full_list.append((x, y))
        print('IMG:', str(picked[-1]))

        if len(picked) > 1:
            # Draw the line
            temp = np.array(picked)
            ax.plot(temp[:, 0], temp[:, 1])
            ax.figure.canvas.draw_idle()

            # Reset list for future pick events
            del picked[0]

    # Display GUI
    print(picked)
    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    file_path = os.path.join(UTILITIES_PATH, "coordinates_2d.txt")
    print(picked_full_list)
    with open (file_path, 'r') as file:
        file_content = file.read()
        coordinates_list = ast.literal_eval(file_content)
    coordinates_list.extend(picked_full_list)
    with open(file_path, 'w') as file:
        file.write(str(coordinates_list))

def calibrate():
    points2D = FileOperations.get_camera_extrinsic_calibration_data(UTILITIES_PATH)
    points3D = FileOperations.get_lidar_extrinsic_calibration_data(UTILITIES_PATH)
    assert points2D.shape[0] == points3D.shape[0], "Number of 2D and 3D points must be the same"
    assert points2D.shape[0] >= 4, "At least 4 points are required"
    assert points2D.shape[1] == 2, "points2D should be of shape (N, 2)"
    assert points3D.shape[1] == 3, "points3D should be of shape (N, 3)"
    # Obtain camera matrix and distortion coefficients
    camera_matrix = CAMERA_MODEL.intrinsicMatrix()
    dist_coeffs = CAMERA_MODEL.distortionCoeffs()

    # Ensure camera matrix is a valid 3x3 matrix
    camera_matrix = np.array(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(dist_coeffs, dtype=np.float64).reshape(-1, 5)
    print(dist_coeffs)
    print(camera_matrix)
    # Estimate extrinsics
    success, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(
        points3D, points2D, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=70.0, confidence=0.99)
    
    if not success:
        print('Initial estimation unsuccessful, skipping refinement')
        return
    
    # Compute re-projection error
    points2D_reproj = cv2.projectPoints(points3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs)[0].squeeze(1)
    assert points2D_reproj.shape == points2D.shape
    error = (points2D_reproj - points2D)[inliers]  # Compute error only over inliers
    error = np.reshape(error, (error.shape[0], 2))
    rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
    print('Re-projection error before LM refinement (RMSE) in px:', str(rmse))
    
    # Refine estimate using LM
    if hasattr(cv2, 'solvePnPRefineLM') and len(inliers) >= 3:
        rotation_vector, translation_vector = cv2.solvePnPRefineLM(
            points3D[inliers], points2D[inliers], camera_matrix, dist_coeffs, rotation_vector, translation_vector)
        points2D_reproj = cv2.projectPoints(points3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs)[0].squeeze(1)
        assert points2D_reproj.shape == points2D.shape
        error = (points2D_reproj - points2D)[inliers]  # Compute error only over inliers
        error = np.reshape(error, (error.shape[0], 2))
        rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
        print('Re-projection error after LM refinement (RMSE) in px:', str(rmse))
    else:
        print('Skipping LM refinement')

    # Convert rotation vector to matrix
    rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
    
    # Save extrinsics
    CUR_PATH = "/home/dhruvagarwal/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities/logitech_extrinsics.npz"
    np.savez(CUR_PATH, R=rotation_matrix, T=translation_vector.T)

    print('Rotation Matrix:', rotation_matrix)
    print('Translation Offsets:', translation_vector.T)


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
    points3D = np.asarray(points3D.tolist())
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
                cv2.fillPoly(mask, [np.array(segmentation_output, np.int32)], 1)
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

# cmap = plt.cm.get_cmap('jet')
# img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
# intensities = np.array([intensity[0] for intensity in intensities])
# print(intensities)
# colors = (cmap(intensities / max_intensity)[:, :3] * 255).astype(np.uint8)
# # colors = colors[:, ::-1]
# # print(colors)
# for i, color in enumerate(colors):
#     color = ( int (color [ 0 ]), int (color [ 1 ]), int (color [ 2 ])) 
#     print(color)
#     cv2.circle(img, tuple(points2D[i]), 2, tuple(color), -1)

def project_both_point_clouds(self):
    dists_realsense, angles_realsense_unmodified, classes_realsense = get_dist_angle_classes(self, self.velodyne_msg, self.realsense_image_msg, self.model_operator, True)
    dists_logitech, angles_logitech_unmodified, classes_logitech = get_dist_angle_classes(self, self.velodyne_msg, self.logitech_image_msg, self.model_operator, False)
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
    

    all_distances = dists_logitech + dists_realsense
    all_angles = angles_logitech + angles_realsense
    all_classes = classes_logitech + classes_realsense
    cartesian_x_distances = all_distances * np.cos(all_angles)
    cartesian_y_distances = all_distances * np.sin(all_angles)
    cones_msg.r = all_distances
    cones_msg.theta = all_angles
    cones_msg.color = all_classes
    self.perception_pub.publish(cones_msg)

    cones_guess = PointCloud()
    positions = []
    for x, y, color_value in zip(cartesian_x_distances, cartesian_y_distances, all_classes):
        if color_value == 2:
            positions.append(Point32())
            positions[-1].x = x
            positions[-1].y = y
            positions[-1].z = 0.0
    cones_guess.points = positions
    cones_guess.header.frame_id = "map"
    cones_guess.header.stamp = self.get_clock().now().to_msg()
    self.perception_visual_pub.publish(cones_guess)

def project_point_cloud(self, velodyne, img_msg, image_pub, model_operator, display_projection, perception_pub, realsense):
    median_distances, angles, chosen_classes = get_dist_angle_classes(self, velodyne, img_msg, model_operator, realsense)
    if display_projection:
        try:
            image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)
    else:
        cones_msg = Cones()
        cones_msg.header.stamp = self.get_clock().now().to_msg()

        print("CONES R: ", median_distances)
        print("CONES THETA: ", angles)
        print("COLOR: ", chosen_classes)

        cartesian_x_distances = median_distances * np.cos(angles)
        cartesian_y_distances = median_distances * np.sin(angles)

        cones_msg.r = median_distances
        cones_msg.theta = angles
        cones_msg.color = chosen_classes
        self.perception_pub.publish(cones_msg)

        cones_guess = PointCloud()
        positions = []
        for x, y, color_value in zip(cartesian_x_distances, cartesian_y_distances, chosen_classes):
            positions.append(Point32())
            positions[-1].x = x
            positions[-1].y = y
            positions[-1].z = 0.0
        cones_guess.points = positions
        cones_guess.header.frame_id = "map"
        cones_guess.header.stamp = self.get_clock().now().to_msg()
        self.perception_visual_pub.publish(cones_guess)

class CalibrateCameraLidar(Node):
    def __init__(self, camera_info, image_color, velodyne_points, calibrate_mode=True, camera_lidar=None, project_mode=False, display_projection = True, realsense = False, combined = False):
        super().__init__('calibrate_camera_lidar')
        logitech_camera_topic = '/sensors/camera/image_color'
        realsense_camera_topic = '/camera/camera/color/image_raw'
        self.project_mode = project_mode
        self.camera_lidar = camera_lidar
        # print(image_color)
        # Subscribe to topic
        self.info_sub = self.create_subscription(CameraInfo, camera_info, self.camera_info_callback, qos_profile_sensor_data)
        if combined:
            self.image_sub_logitech = self.create_subscription(Image, logitech_camera_topic, self.logitech_callback, qos_profile_sensor_data)
            self.image_sub_realsense = self.create_subscription(Image, realsense_camera_topic, self.realsense_callback, qos_profile_sensor_data)
        else:
            self.image_sub = self.create_subscription(Image, image_color, self.image_callback, qos_profile_sensor_data)
        self.velodyne_sub = self.create_subscription(PointCloud2, velodyne_points, self.velodyne_callback, qos_profile_sensor_data)
        # Publish output topic
        self.image_pub = self.create_publisher(Image, camera_lidar, 5) if camera_lidar else None
        self.perception_pub = self.create_publisher(Cones, '/perception_cones', 1)
        self.perception_visual_pub = self.create_publisher(PointCloud, '/perception_cones_viz', 1)
        self.camera_info_msg = None
        self.image_msg = None
        self.realsense_image_msg = None
        self.logitech_image_msg = None
        self.velodyne_msg = None
        self.calibrate_mode = calibrate_mode
        self.display_projection = display_projection
        self.realsense = realsense
        self.combined = combined
        self.model_operator = ModelOperations(UTILITIES_PATH)
    def camera_info_callback(self, msg):
        self.camera_info_msg = msg
        self.process()

    def realsense_callback(self, msg):
        self.realsense_image_msg = msg
        self.process()

    def logitech_callback(self, msg):
        self.logitech_image_msg = msg
        self.process() 

    def image_callback(self, msg):
        self.image_msg = msg
        self.process()

    def velodyne_callback(self, msg):
        self.velodyne_msg = msg
        self.process()

    def process(self):
        global FIRST_TIME, PAUSE, TF_BUFFER, TF_LISTENER, CAMERA_MODEL
        self.camera_info_msg = CameraInfo()
        if self.camera_info_msg and (self.image_msg or self.realsense_image_msg) and self.velodyne_msg:
            if FIRST_TIME:
                FIRST_TIME = False
                # Setup camera model
                self.get_logger().info('Setting up camera model')
                camera_info_msg = CameraInfo()
                camera_info_msg.header.frame_id = 'camera_frame'
                camera_info_msg.distortion_model = 'plumb_bob'
                h, w, d, k, p = FileOperations.get_intrinsic_parameters(UTILITIES_PATH, self.realsense)
                camera_info_msg.height = int(h)
                camera_info_msg.width = int(w)
                camera_info_msg.d = d
                camera_info_msg.k = k
                # camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                camera_info_msg.p = p
                #  = [493.53473414, -3.42710262, 286.5538899, 0.0, 0.0, 490.94207904, 225.36415318, 0.0, 0.0, 0.0, 1.0, 0.0]
                self.camera_info_msg = camera_info_msg
                CAMERA_MODEL.fromCameraInfo(self.camera_info_msg)

                # TF listener
                self.get_logger().info('Setting up static transform listener')
                TF_BUFFER = tf2_ros.Buffer()
                TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER, self)
                self.R, self.T = FileOperations.get_extrinsic_parameters(UTILITIES_PATH)
            # Projection/display mode
            if self.combined:
                project_both_point_clouds(self)
            elif self.calibrate_mode and self.project_mode:
                calibrate()
            elif self.project_mode:
                project_point_cloud(self, self.velodyne_msg, self.image_msg, self.image_pub, self.model_operator, self.display_projection, self.perception_pub, self.realsense)
            else:
                # Create GUI processes
                now = self.get_clock().now().seconds_nanoseconds()[0]
                img_p = multiprocessing.Process(target=extract_points_2D, args=[self.image_msg, now])
                img_p.start()
                img_p.join()
            	


def main(args=None):
    rclpy.init(args=args)
    print("in here")
    
    # Default values
    camera_info = '/sensors/camera/camera_info'  # Ignore this one
    image_color = '/sensors/camera/image_color'
    # image_color = '/sensors/camera/image_color'  # The camera topic
    velodyne_points = '/rslidar_points'  # The LiDAR topic
    camera_lidar = '/sensors/camera/camera_lidar'  # The output topic
    calibrate_mode = False
    project_mode = True
    display_projection = False
    realsense = False
    combined = False
    
    print(sys.argv)
    if len(sys.argv) == 1:
        pass  # Use the default values set above
    elif sys.argv[1] == '--select-points':
        project_mode = False
    elif sys.argv[1] == '--select-points-realsense':
        camera_info = '/camera/camera/color/camera_info'  # Ignore this one
        image_color = '/camera/camera/color/image_raw'
        realsense = True
        project_mode = False
    elif sys.argv[1] == '--calibration':
        calibrate_mode = True
    elif sys.argv[1] == '--calibration-realsense':
        camera_info = '/camera/camera/color/camera_info'  # Ignore this one
        image_color = '/camera/camera/color/image_raw'
        calibrate_mode = True
        realsense = True
    elif sys.argv[1] == '--projection':
        display_projection = True
    elif sys.argv[1] == '--projection-realsense':
        camera_info = '/camera/camera/color/camera_info'  # Ignore this one
        image_color = '/camera/camera/color/image_raw'
        display_projection = True
        realsense = True
    elif sys.argv[1] == '--projection-combined':
        combined = True

    # Initialize the node with the (now guaranteed) variables
    node = CalibrateCameraLidar(camera_info, image_color, velodyne_points, calibrate_mode, camera_lidar, project_mode, display_projection, realsense, combined)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
