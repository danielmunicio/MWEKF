import numpy as np 
import matplotlib.pyplot as plt 
import rclpy 
from rclpy.node import Node

def plot_values(self):
    if self.camera_cones_map is None or self.lidar_cones_map is None:
        self.get_logger().warn("Cone maps not fully received yet.")
        return

    plt.figure(figsize=(8, 8))
    # Plot lidar cones in red
    plt.scatter(
        self.lidar_cones_map[0, :],  # x
        self.lidar_cones_map[1, :],  # y
        c='r', label='Lidar Cones', alpha=0.7
    )
    # Plot camera cones in blue
    plt.scatter(
        self.camera_cones_map[0, :],  # x
        self.camera_cones_map[1, :],  # y
        c='b', label='Camera Cones', alpha=0.7
    )
    plt.title("Lidar vs Camera Cone Detection")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_icp(self):
    pass


def run_icp(self):
    print("LIDAR CONES: ", self.lidar_cones_map)
    print("CAMERA CONES: ", self.camera_cones_map)
    pass
    #first_match = initial_matching()


def match_nn(lidar_cones, camera_cones, num_neighbors):
    """
    lidar_cones: nx3 array of [x, y, -1]
    camera_cones: nx3 array of [x, y, color]
    """
    for cone in camera_cones:
        # Get distance from each cone in camera_cones to lidar_cones
        camera_cone_distances = np.linalg.norm(lidar_cones[:, :2]- cone[:2])
        print("Distances: ", camera_cone_distances)
        # Get distance from each cone in camera_cones to lidar_cones
        np.linalg.norm([])
