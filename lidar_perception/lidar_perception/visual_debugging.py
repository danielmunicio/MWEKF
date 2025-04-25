from rclpy.node import Node
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
import sensor_msgs_py.point_cloud2 as pc2

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
