#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from eufs_msgs.msg import ConeArrayWithCovariance
from all_settings.all_settings import SimulatorPerceptionSettings as settings
from time import sleep

class ConeNoiseSimulator(Node):
    def __init__(self):
        super().__init__('cone_noise_simulator')
        
        self.camera_msg = None
        self.lidar_msg = None

        # Publishers and Subscribers
        self.ground_truth_sub = self.create_subscription(ConeArrayWithCovariance, '/ground_truth/cones', self.cone_callback, 1)
        self.camera_cones_pub = self.create_publisher(ConeArrayWithCovariance, '/camera/cones', 1)
        self.lidar_cones_pub = self.create_publisher(ConeArrayWithCovariance, '/lidar/cones', 1)

        # Mimics publishing at the camera and LiDAR's given rates
        self.camera_timer = self.create_timer(1/settings.camera_hz, self.publish_camera)
        self.lidar_timer = self.create_timer(1/settings.lidar_hz, self.publish_lidar)

    def add_gaussian_noise(self, x, y, is_camera):
        """Add Gaussian noise to x, y coordinates."""
        if is_camera: 
            x += np.random.normal(0, settings.camera_noise_std)
            y += np.random.normal(0, settings.camera_noise_std)
        else:
            x += np.random.normal(0, settings.lidar_noise_std)
            y += np.random.normal(0, settings.lidar_noise_std)
        return x, y

    def cone_callback(self, msg):
        # Adds noise to cones, and keeps them ready to publish
        # Doesn't actually publish though, publishing controlled by a timer 
        camera_msg = ConeArrayWithCovariance()
        lidar_msg = ConeArrayWithCovariance()

        for cone in msg.blue_cones:
            camera_x, camera_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=True)
            cone.point.x = camera_x
            cone.point.y = camera_y
            camera_msg.blue_cones.append(cone)
            self.camera_msg = camera_msg

            lidar_x, lidar_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=False)
            cone.point.x = lidar_x
            cone.point.y = lidar_y
            lidar_msg.blue_cones.append(cone)
            self.lidar_msg = lidar_msg

        for cone in msg.yellow_cones:
            camera_x, camera_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=True)
            cone.point.x = camera_x
            cone.point.y = camera_y
            camera_msg.yellow_cones.append(cone)
            self.camera_msg = camera_msg

            lidar_x, lidar_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=False)
            cone.point.x = lidar_x
            cone.point.y = lidar_y
            lidar_msg.yellow_cones.append(cone)
            self.lidar_msg = lidar_msg

        for cone in msg.orange_cones:
            camera_x, camera_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=True)
            cone.point.x = camera_x
            cone.point.y = camera_y
            camera_msg.orange_cones.append(cone)
            self.camera_msg = camera_msg

            lidar_x, lidar_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=False)
            cone.point.x = lidar_x
            cone.point.y = lidar_y
            lidar_msg.orange_cones.append(cone)
            self.lidar_msg = lidar_msg

        for cone in msg.big_orange_cones:
            camera_x, camera_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=True)
            cone.point.x = camera_x
            cone.point.y = camera_y
            camera_msg.big_orange_cones.append(cone)
            self.camera_msg = camera_msg

            lidar_x, lidar_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=False)
            cone.point.x = lidar_x
            cone.point.y = lidar_y
            lidar_msg.big_orange_cones.append(cone)
            self.lidar_msg = lidar_msg

    def publish_camera(self):
        if self.camera_msg is None:
            return
        sleep(max(0, np.random.normal(settings.camera_delay_mean, settings.camera_delay_std)))
        self.camera_cones_pub.publish(self.camera_msg)
    
    def publish_lidar(self):
        if self.lidar_msg is None:
            return
        sleep(max(0, settings.lidar_delay_mean, settings.lidar_delay_std))
        self.lidar_cones_pub.publish(self.lidar_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConeNoiseSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 