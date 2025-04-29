from rclpy.node import Node
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
import sensor_msgs_py.point_cloud2 as pc2
import ros2_numpy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation as R

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


def publish_raw_pointcloud(self, filtered_points, header):
    msg = ros2_numpy.point_cloud2.array_to_point_cloud2(
        np.core.records.fromarrays(
            filtered_points.T,
            names='x,y,z,intensity',
            formats='f4,f4,f4,f4'
        ),
        frame_id=header.frame_id
    )
    self.filtered_pub.publish(msg)


def publish_ground_plane_marker(self):
    marker = Marker()
    marker.header.frame_id = "map"  # Make sure this matches your TF frames
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = "ground_plane"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    center = np.array([0.0, 0.0, -self.d / self.c])
    normal = np.array([self.a, self.b, self.c])
    normal = normal / np.linalg.norm(normal)
    z_axis = np.array([0, 0, 1])
    rotation_vector = np.cross(z_axis, normal)
    angle = np.arccos(np.clip(np.dot(z_axis, normal), -1.0, 1.0))

    if np.linalg.norm(rotation_vector) < 1e-6:
        quat = R.from_rotvec([0, 0, 0]).as_quat()
    else:
        quat = R.from_rotvec(rotation_vector / np.linalg.norm(rotation_vector) * angle).as_quat()

    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.scale.x = 20.0
    marker.scale.y = 10.0
    marker.scale.z = 0.01

    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
    self.marker_pub.publish(marker)
