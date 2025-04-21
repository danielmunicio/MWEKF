#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from eufs_msgs.msg import ConeArrayWithCovariance

class ConeNoiseSimulator(Node):
    def __init__(self):
        super().__init__('cone_noise_simulator')
        
        self.position_std_dev = 0.1  # 10cm standard deviation
        
        self.ground_truth_sub = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones',
            self.cone_callback,
            10
        )
        
        self.noisy_cones_pub = self.create_publisher(
            ConeArrayWithCovariance,
            '/camera/cones',
            10
        )
        

    def add_gaussian_noise(self, x, y):
        """Add Gaussian noise to x, y coordinates."""
        noise_x = np.random.normal(0, self.position_std_dev)
        noise_y = np.random.normal(0, self.position_std_dev)
        return x + noise_x, y + noise_y

    def cone_callback(self, msg):
        """Callback for ground truth cones - adds noise and republishes."""
        noisy_msg = ConeArrayWithCovariance()
        noisy_msg.header = msg.header
        
        for cone in msg.blue_cones:
            noisy_x, noisy_y = self.add_gaussian_noise(cone.point.x, cone.point.y)
            cone.point.x = noisy_x
            cone.point.y = noisy_y
            noisy_msg.blue_cones.append(cone)
        
        for cone in msg.yellow_cones:
            noisy_x, noisy_y = self.add_gaussian_noise(cone.point.x, cone.point.y)
            cone.point.x = noisy_x
            cone.point.y = noisy_y
            noisy_msg.yellow_cones.append(cone)
        
        for cone in msg.orange_cones:
            noisy_x, noisy_y = self.add_gaussian_noise(cone.point.x, cone.point.y)
            cone.point.x = noisy_x
            cone.point.y = noisy_y
            noisy_msg.orange_cones.append(cone)
        
        for cone in msg.big_orange_cones:
            noisy_x, noisy_y = self.add_gaussian_noise(cone.point.x, cone.point.y)
            cone.point.x = noisy_x
            cone.point.y = noisy_y
            noisy_msg.big_orange_cones.append(cone)
        
        self.noisy_cones_pub.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConeNoiseSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 