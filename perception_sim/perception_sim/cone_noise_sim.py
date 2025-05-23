import rclpy
from rclpy.node import Node
import numpy as np
from eufs_msgs.msg import ConeArrayWithCovariance, CarState
from feb_msgs.msg import ConesCartesian
from all_settings.all_settings import SimulatorPerceptionSettings as settings
from time import sleep
import math
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class ConeNoiseSimulator(Node):
    def __init__(self):
        super().__init__('cone_noise_simulator')
        
        self.camera_msg = None
        self.lidar_msg = None
        self.state = np.array([0., 0., 0.]) # x y heading
        # Publishers and Subscribers
        self.ground_truth_sub = self.create_subscription(ConeArrayWithCovariance, '/ground_truth/cones', self.cone_callback, 1)
        self.camera_cones_pub = self.create_publisher(ConesCartesian, '/camera/cones', 1)
        self.lidar_cones_pub = self.create_publisher(ConesCartesian, '/lidar/cones', 1)
        # Mimics publishing at the camera and LiDAR's given rates
        self.camera_timer = self.create_timer(1/settings.camera_hz, self.publish_camera)
        self.lidar_timer = self.create_timer(1/settings.lidar_hz, self.publish_lidar)
        self.cones_camera_vis_pub = self.create_publisher(PointCloud, '/cones_noise_viz', 1)       

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
        camera_msg = ConesCartesian()
        lidar_msg = ConesCartesian()

        # Format: (cones, camera_color, lidar_color)
        cone_types = [
            (msg.blue_cones, 2, -1),
            (msg.yellow_cones, 1, -1),
        ]

        for cones, camera_color, lidar_color in cone_types:
            for cone in cones:
                camera_x, camera_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=True)
                if camera_x > 0.1:
                    camera_msg.x.append(camera_x)
                    camera_msg.y.append(camera_y)
                    camera_msg.color.append(camera_color)

                lidar_x, lidar_y = self.add_gaussian_noise(cone.point.x, cone.point.y, is_camera=False)
                lidar_msg.x.append(lidar_x)
                lidar_msg.y.append(lidar_y)
                lidar_msg.color.append(-1)

        self.camera_msg = camera_msg
        self.lidar_msg = lidar_msg

    def publish_camera(self):
        if self.camera_msg is None:
            return
        sleep(max(0, np.random.normal(settings.camera_delay_mean, settings.camera_delay_std)))
        self.camera_cones_pub.publish(self.camera_msg)

        # Publish noisy camera cones as PointCloud for RViz
        pointcloud_msg = PointCloud()
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        pointcloud_msg.header.frame_id = "base_footprint"  # or "base_link" depending on your TF setup

        for x, y in zip(self.camera_msg.x, self.camera_msg.y):
            pointcloud_msg.points.append(Point32(x=x, y=y, z=0.0))

        self.cones_camera_vis_pub.publish(pointcloud_msg)
    
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