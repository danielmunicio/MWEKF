import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from mpl_toolkits.mplot3d import Axes3D

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
        if self.recieved: 
            return

        # Read points with intensity
        points = list(pc2.read_points(
            pointcloud,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        ))

        # Filter points based on plane distance
        filtered_points = self.filter_points_by_plane_and_distance(points)
        #filtered_points = self.filter_points_by_distance(filtered_points)

        #filtered_points = np.array(filtered_points, dtype=np.float64)
        filtered_points = np.array([list(p) for p in filtered_points], dtype=np.float64)
        self.get_logger().info(f"Original: {len(points)} pts, Filtered: {len(filtered_points)} pts")

        # Do a sklearn DB Scan 
        # EPS is like the max distance from a point to another point in a cluster, to add point to cluster
        # Min samples is minimum number of points in a cluster to be a cluster
        clustered_points_scan = DBSCAN(eps=0.1, min_samples=30).fit(filtered_points[:, :3])

        #labels = clustered_points_scan.labels_

        ## Number of clusters in labels, ignoring noise if present.
        #n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        #n_noise_ = list(labels).count(-1)

        #print("Estimated number of clusters: %d" % n_clusters_)
        #print("Estimated number of noise points: %d" % n_noise_)

        filtered_cone_labels = self.filter_cluster_for_cones(filtered_points, clustered_points_scan.labels_)
        cone_mask = np.isin(clustered_points_scan.labels_, filtered_cone_labels)
        cone_points = filtered_points[cone_mask]

        #self.plot_clusters_3d(filtered_points[:, :3], clustered_points_scan.labels_)

        # Publish filtered pointcloud
        self.publish_filtered_pointcloud(cone_points, pointcloud.header)


    def filter_points_by_plane_and_distance(self, points, threshold=0.05, max_distance=10.0):
        """
        Filters out points:
        - Close to the plane described by the equation (within `threshold`)
        - Farther than `max_distance` meters from the origin
        """
        filtered_points = []
        for p in points:
            x, y, z = p[0], p[1], p[2]
            
            # Compute signed distance from the plane
            distance_to_plane = abs(a * x + b * y + c * z + d) / plane_normal
            
            # Compute Euclidean distance from origin
            distance_from_origin = (x**2 + y**2 + z**2) ** 0.5

            # Keep only points that are not too close to the plane and not too far away
            if distance_to_plane > threshold and distance_from_origin <= max_distance:
                filtered_points.append(p)

        return filtered_points



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

            print("X Size: ", max_values[0] - min_values[0])
            print("Y Size: ", max_values[1] - min_values[1])
            print("Z Size: ", max_values[2] - min_values[2])


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
