import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from mpl_toolkits.mplot3d import Axes3D
from time import perf_counter
# Plane coefficients from the equation
a, b, c, d = -0.055, -0.003, 0.998, 0.591
plane_normal = np.sqrt(a**2 + b**2 + c**2)

class LiDARCones(Node):
    def __init__(self):
        super().__init__('lidar_only')
        self.z_values = None
        self.recieved = False
        self.lidar_sub = self.create_subscription(PointCloud2, 'rslidar_points', self.lidar_callback, 1)
        self.filtered_pub = self.create_publisher(PointCloud2, '/filtered_pointcloud', 1)

    def lidar_callback(self, pointcloud):
        # Read points with intensity
        start = perf_counter()
        points = np.asarray(list(pc2.read_points(
            pointcloud,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        )))

        # Convert structured array to plain float array
        points = np.vstack([points['x'], points['y'], points['z'], points['intensity']]).T

        extract_points = perf_counter()
        # Filter points based on plane distance
        filtered_points = self.filter_points_by_plane_and_distance(points)
        filter_points = perf_counter()
        self.get_logger().info(f"Original: {len(points)} pts, Filtered: {len(filtered_points)} pts")

        # Do a sklearn DB Scan 
        # EPS is like the max distance from a point to another point in a cluster, to add point to cluster
        # Min samples is minimum number of points in a cluster to be a cluster
        clustered_points_scan = DBSCAN(eps=0.1, min_samples=30).fit(filtered_points[:, :3])
        cluster_points = perf_counter()

        filtered_cone_labels = self.filter_cluster_for_cones(filtered_points, clustered_points_scan.labels_)
        find_cones = perf_counter()
        cone_mask = np.isin(clustered_points_scan.labels_, filtered_cone_labels)
        cone_points = filtered_points[cone_mask]
        
        #self.plot_clusters_3d(filtered_points[:, :3], clustered_points_scan.labels_)

        # Publish filtered pointcloud
        print("Overall Times: ")
        print("Extracting Pointcloud: ", extract_points - start)
        print("Filtering Points: ", filtered_points - extract_points)
        print("Clustering Points: ", cluster_points - filtered_points)
        print("Finding Cones From Clusters: ", find_cones - cluster_points)

        self.publish_filtered_pointcloud(cone_points, pointcloud.header)


    def filter_points_by_plane_and_distance(self, points, threshold=0.05, max_distance=10.0):
        """
        Vectorized filtering of points based on:
        - Distance from a plane
        - Distance from the origin
        """
        # Plane parameters (assumed to be defined somewhere globally or in the class)
        # Plane equation: ax + by + cz + d = 0
  

        xyz = points[:, :3]  # Shape (N, 3)
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        # Compute signed distance from the plane (vectorized)
        distance_to_plane = np.abs(a * x + b * y + c * z + d) / plane_normal

        # Compute Euclidean distance from origin (vectorized)
        distance_from_origin = np.sqrt(x**2 + y**2 + z**2)

        # Create boolean mask for filtering
        mask = (distance_to_plane > threshold) & (distance_from_origin <= max_distance)

        return points[mask]

    def publish_filtered_pointcloud(self, points, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.filtered_pub.publish(cloud_msg)
        self.get_logger().info("Filtered point cloud published to /filtered_pointcloud")

    def filter_cluster_for_cones(self, points, labels):
        cones = []
        unique_labels = set(labels) - {-1}
        for label in unique_labels:
            cluster = points[labels == label]
            min_values = cluster.min(axis=0)
            max_values = cluster.max(axis=0)
            print(f"Bounding Box {label}")
            x_box = max_values[0] - min_values[0]
            y_box = max_values[1] - min_values[1]
            z_box = max_values[2] - min_values[2]

            # Arbitrary filtering status from me for now >:) 
            if (x_box < 0.25 and y_box < 0.15 and z_box < 0.3):
                cones.append(label)
        return cones
        
    def plot_clusters_3d(self, points, labels):
        """
        Plots clustered 3D points using matplotlib, excluding noise points.
    
        Args:
            points (np.ndarray): Nx3 array of 3D points.
            labels (np.ndarray): Cluster labels for each point.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Get unique labels excluding noise (-1)
        unique_labels = set(labels) - {-1}

        # Assign a color to each cluster
        colors = plt.cm.get_cmap('tab20', len(unique_labels))

        for k in unique_labels:
            class_member_mask = (labels == k)
            xyz = points[class_member_mask]

            ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=[colors(k)], label=f'cluster {k}', s=10)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.title("3D Clusters from DBSCAN")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = LiDARCones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
