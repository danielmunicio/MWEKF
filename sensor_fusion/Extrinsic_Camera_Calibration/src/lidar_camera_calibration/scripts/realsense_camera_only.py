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
UTILITIES_PATH = '/home/daniel/feb-system-integration/sensor_fusion/Extrinsic_Camera_Calibration/src/lidar_camera_calibration/utilities'#os.path.join(PKG_PATH, 'utilities')

INTRINSIC_MATRIX = np.array([607.3717041015625,  0.0, 321.06414794921875, 0.0, 607.5971069335938, 239.82249450683594,  0.0, 0.0, 1.0]).reshape((3, 3))

def publish_cones_distances(self):

    # Apply segmentation and draw points within the segmented regionsi
    img = np.reshape(CV_BRIDGE.imgmsg_to_cv2(self.image_msg, 'bgr8'), (480, 640, 3))
    # print("SHAPE 2: ", img)

    segmentation_outputs, classes, conf = self.model_operator.predict2(img)
    median_distances = []
    center_points = []
    angles = []
    included_classes = []
    distances = np.reshape(self.distance_msg.data[::2], (480, 640))
    img_height = 480
    img_width = 640
    # print(np.array(distances))
    # print("SHAPE: ", np.shape(np.array(distances)))
    # print("number of cones detected", len(segmentation_outputs))
    for idx, segmentation_output in enumerate(segmentation_outputs):
        if conf[idx] > 0.7:
            mask = np.zeros((img_height, img_width), dtype=np.uint8)
            if len(segmentation_output) != 0:
                # startTime = time.perf_counter()
                segmentation_output = np.array(segmentation_output, np.int32)
                for idx2 in range(len(segmentation_output)):
                    segmentation_output[idx2] = (segmentation_output[idx2][1], segmentation_output[idx2][0])
                cv2.fillPoly(mask, [segmentation_output], 1)
                # Create a colored version of the mask to overlay
                # plt.figure(figsize=(5, 5))
                # plt.title('Mask')
                # plt.imshow(mask, cmap='gray')  # Use grayscale colormap
                # plt.axis('off')
                # plt.show()


                print(segmentation_output)
                # Use the boolean array to filter points3D
                points_in_polygon = []
                x_store = []
                y_store = []
                for i in range(len(mask)):
                    for j in range(len(mask[i])):
                        if mask[i][j] == 1:
                            points_in_polygon.append(distances[i][j])
                            x_store.append(i)
                            y_store.append(j)
                # points_in_polygon = distanc es[mask == 1]
                # print(points_in_polygon)
                print("length", len(points_in_polygon))
                # angle = np.arctan(np.median(points_in_polygon[:, 0])/np.median(points_in_polygon[:, 1]))
                # if angle < 0:
                #     angle = -((np.pi / 2) + angle)
                # else:
                #     angle = (np.pi / 2) - angle
                # angles.append(-angle)
                # distances = np.linalg.norm(points_in_polygon)
                if len(points_in_polygon) > 0:
                    intrinsic_matrix = np.array([[607.3717041015625, 0, 321.06414794921875],
                             [0, 607.5971069335938, 239.82249450683594],
                             [0, 0, 1]])
                    pixel_coords = (np.median(x_store), np.median(y_store))
                    median_distance = np.median(points_in_polygon) if len(points_in_polygon) > 0 else float('nan')
                    median_distances.append(median_distance)
                    included_classes.append(classes[idx])
                    fx = intrinsic_matrix[0, 0]  # Focal length in x direction
                    fy = intrinsic_matrix[1, 1]  # Focal length in y direction
                    cx = intrinsic_matrix[0, 2]  # Principal point x
                    cy = intrinsic_matrix[1, 2]  # Principal point y

                    # Get pixel coordinates
                    x_pixel, y_pixel = pixel_coords

                    # Convert pixel coordinates to normalized camera coordinates (u, v)
                    # u = (x - cx) / fx
                    # v = (y - cy) / fy
                    u = (x_pixel - cx) / fx
                    v = (y_pixel - cy) / fy

                    # Compute 3D coordinates in the camera frame
                    # X = u * z, Y = v * z, Z = z
                    X = u * median_distance
                    Y = v * median_distance
                    Z = median_distance

                    # Now, we can compute the angle to the point using arctan
                    # Angle in the X-Z plane (horizontal angle)
                    theta_x = np.arctan2(X, Z)

                    # Angle in the Y-Z plane (vertical angle)
                    theta_y = np.arctan2(Y, Z)

                    angles.append(theta_x)
        cones_msg = Cones()
        print(included_classes)
        #orange = 0, yellow = 1, blue = 2 in feb system
        classesToActual = {0: 0, 1: 1, 2: 7, 3: 8, 4: 9}
        yolo_class_to_feb_class = {8: 2, 1: 1, 0: 0, 7: 0, 9: 0}
        mask_conf = [idx for idx, con in enumerate(conf) if con > 0.7]
        print(len(mask_conf))
        print("================")
        chosen_classes = []
        # for i in range(included_classes):
        #     print(included_classes[i])
        #     print(classesToActual[int(included_classes[i])])
        #     print(yolo_class_to_feb_class[classesToActual[int(included_classes[i])]])
        chosen_classes = [yolo_class_to_feb_class[classesToActual[int(included_classes[i])]] for i in range(len(included_classes))]
        # chosen_angles = []
        # chosen_angles = [angles[i] for i in mask_conf]
        # chosen_distances = median_distances
        cones_msg.header.stamp = self.get_clock().now().to_msg()
        cones_msg.r = median_distances
        cones_msg.theta = angles
        cones_msg.color = chosen_classes
        self.perception_pub.publish(cones_msg)


class Realsense(Node):
    def __init__(self, camera_color, distance_raw):
        print('starting')
        super().__init__('calibrate_camera_lidar')

        # self.project_mode = project_mode
        # self.camera_lidar = camera_lidar

        # Subscribe to topic
        self.distance_msg = None
        self.image_msg = None
        self.image_sub = self.create_subscription(Image, camera_color, self.image_callback, qos_profile_sensor_data)
        self.distance_sub = self.create_subscription(Image, distance_raw, self.distance_callback, qos_profile_sensor_data)

        self.perception_pub = self.create_publisher(Cones, '/perception_cones', 1)
        self.model_operator = ModelOperations(UTILITIES_PATH)

    def image_callback(self, msg):
        self.image_msg = msg
        self.process()

    def distance_callback(self, msg):
        self.distance_msg = msg
        print(len(msg.data))
        self.process()

    def process(self):
        global FIRST_TIME, PAUSE, TF_BUFFER, TF_LISTENER, CAMERA_MODEL
        if self.distance_msg and self.image_msg:
            publish_cones_distances(self)
            	


def main(args=None):
    rclpy.init(args=args)
    node = None
    if len(sys.argv) == 1:
        camera_color = '/camera/camera/color/image_raw' #camera color topic
        distance_raw = '/camera/camera/aligned_depth_to_color/image_raw' #distance topic
    node = Realsense(camera_color, distance_raw)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()