import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2A
from feb_msgs.msg import ConesCartesian
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from mpl_toolkits.mplot3d import Axes3D
from time import perf_counter
import ros2_numpy
from cuml.cluster import DBSCAN as cuDBSCAN
import cupy as cp
from all_settings.all_settings import LiDAROnlySettings as settings
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
        self.perception_pub = self.create_publisher(ConesCartesian, '/cones/lidar', 1)
        self.perception_vis_pub = self.create_publisher(PointCloud, 'cones/viz/lidar', 1)

    def lidar_callback(self, pointcloud):
        # Read points with intensity
        start = perf_counter()
        points_xyz = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['xyz']
        intensities = ros2_numpy.point_cloud2.point_cloud2_to_array(pointcloud)['intensity']

        points = np.hstack([points_xyz, intensities])
        points = points[~np.isnan(points).any(axis=1)]

        extract_points = perf_counter()
        # Filter points based on plane distance
        filtered_points = self.filter_points_by_plane_and_distance(points,
                                                                    threshold=settings.ground_filter_threshold,
                                                                    max_distance=settings.max_distance
                                                                    )

        filter_points = perf_counter()
        self.get_logger().info(f"Original: {len(points)} pts, Filtered: {len(filtered_points)} pts")

        # Do a sklearn DB Scan 
        # EPS is like the max distance from a point to another point in a cluster, to add point to cluster
        # Min samples is minimum number of points in a cluster to be a cluster
        #clustered_points_scan = DBSCAN(eps=0.1, min_samples=30, n_jobs=-1).fit(filtered_points[:, :3])

        # Convert numpy array to cupy (GPU array)
        filtered_points_gpu = cp.asarray(filtered_points[:, :3])

        # Run cuML DBSCAN
        clustered_points_scan = cuDBSCAN(eps=settings.eps, min_samples=settings.min_samples).fit(filtered_points_gpu)
        labels_cp = clustered_points_scan.labels_
        labels_np = labels_cp.get() if isinstance(labels_cp, cp.ndarray) else labels_cp
        cluster_points = perf_counter()

        #filtered_cone_labels = self.filter_cluster_for_cones(filtered_points, clustered_points_scan.labels_)
        filtered_cone_labels = self.filter_cluster_for_cones_cuML(filtered_points, labels_cp)
        filtered_cone_labels_gpu = cp.asarray(filtered_cone_labels)
        find_cones = perf_counter()

        print("FILTERED CONE LABELS: ", filtered_cone_labels)

        positions = []
        cone_msg = ConesCartesian()
        cones_guess = PointCloud()
        for label in filtered_cone_labels:
            #print(f"Cone cluster label {label} has {len(individual_cone_points)} points")
            mask = (labels_np == label)
            individual_cone_points = filtered_points[mask]
            cone_pose = self.find_cone_position(individual_cone_points)
            cone_msg.x.append(cone_pose[0])
            cone_msg.y.append(cone_pose[1])
            cone_msg.color.append(-1)
            cone_msg.header.stamp = self.get_clock().now().to_msg()

            # Perception Vis Publishers
            pt = Point32()
            print("CONE POSE: ", cone_pose)
            print("TYPE: ", type(cone_pose[0]))
            pt.x = float(cone_pose[0])
            pt.y = float(cone_pose[1])
            pt.z = 0.0
            positions.append(pt)

        self.perception_pub.publish(cone_msg)

        cones_guess.points = positions
        cones_guess.header.frame_id = "map"
        cones_guess.header.stamp = self.get_clock().now().to_msg()
        self.perception_vis_pub.publish(cones_guess)

        # NOTE: Used to work, can be returned on in like 5 minutes to confirm pointclouds filtering work

        #self.publish_filtered_pointcloud(individual_cone_points, pointcloud.header)

        #cone_mask = np.isin(clustered_points_scan.labels_, filtered_cone_labels)
        #cone_mask = cp.isin(clustered_points_scan.labels_, filtered_cone_labels_gpu)

        #cone_mask_np = cone_mask.get()  # Convert cuPy array to NumPy array
        #cone_points = filtered_points[cone_mask]
        
        #cone_points = filtered_points[cone_mask_np]

        #print("CONE POINTS: ", cone_points)

        #self.plot_clusters_3d(filtered_points[:, :3], clustered_points_scan.labels_)
        finish = perf_counter()
        # Publish filtered pointcloud
        print("Overall Times: ")
        print("Extracting Pointcloud: ", extract_points - start)
        print("Filtering Points: ", filter_points - extract_points)
        print("Clustering Points: ", cluster_points - filter_points)
        print("Finding Cones From Clusters: ", find_cones - cluster_points)
        print("Total Time: ", finish - start)


    def filter_points_by_plane_and_distance(self, points, threshold=0.05, max_distance=10.0):
        """
        Vectorized filtering of points based on:
        - Distance from a plane
        - Distance from the origin
        """
        # Plane equation: ax + by + cz + d = 0
        a, b, c, d = settings.ground_plane_coefficents

        xyz = points[:, :3]  # Shape (N, 3)
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        distance_to_plane = np.abs(a * x + b * y + c * z + d) / plane_normal

        distance_from_origin = np.sqrt(x**2 + y**2 + z**2)

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
    

    def filter_cluster_for_cones_cuML(self, points, labels):
        cones = []
        # Convert cuPy labels to NumPy for indexing
        labels_np = cp.asnumpy(labels)

        # Get unique labels excluding -1 (noise label)
        unique_labels = set(labels_np[labels_np != -1])
        
        for label in unique_labels:
            # Extract the points corresponding to the current cluster label
            cluster = points[labels_np == label]
            
            # Convert the cluster points to NumPy for min/max calculation
            cluster_np = cp.asnumpy(cluster)  # Convert to NumPy for min/max, as cuPy might be slower here
            
            # Get the bounding box
            min_values = cluster_np.min(axis=0)
            max_values = cluster_np.max(axis=0)
            print(f"Bounding Box {label}")
            
            # Check if these fit insize criteria
            x_box = settings.x_size[0] < max_values[0] - min_values[0] < settings.x_size[1]
            y_box = settings.y_size[0] < max_values[1] - min_values[1] < settings.y_size[1]
            z_box = settings.z_size[0] < max_values[2] - min_values[2] < settings.z_size[1]

            if (x_box and y_box and z_box):
                cones.append(label)
        
        return cones

    def find_cone_position(self, pointcloud, method='median'):
        """
        Find the position of a cone, given its pointcloud. For now only returns x and y position

        Args: 
            pointcloud (numpy array): Nx3 array of 3D Points
        """
        if method == 'median':
            cone_positions = np.median(pointcloud, axis=0)
        if method == 'mean':
            cone_positions = np.mean(pointcloud, axis=0)

        return cone_positions[0:2]

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
