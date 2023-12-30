from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import rclpy
import random
import time
import matplotlib.pyplot as plt


class LidarNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_node')
        
        # SUBSCRIBERS
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.lidar_callback,
            10)
        
        # PUBLISHERS
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/perception/lidar/filtered_cloud',
            1)
        
        # Global varibles
        self.mount_height = 0.6
        self.upper_threshold = 0.3
        self.max_z = self.upper_threshold - self.mount_height
        self.max_range = 15

    def lidar_callback(self, msg: PointCloud2) -> None:
        # Convert PointCloud2 message to a numpy array
        start = time.perf_counter()
        pc_data = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width, len(msg.fields)))
        end = time.perf_counter()
        conv_time = end - start
        
        num_range_bins = 10
        z_vals = np.linspace(-1., 0., 50)
        r_vals = np.linspace(0., 20., num_range_bins)
        mask = np.zeros((msg.height, msg.width), dtype=bool)
        
        # Get the index of the 'z' field
        z_index = next(i for i, field in enumerate(msg.fields) if field.name == 'z')
        for channel in range(5):
            channel_mask = np.zeros(msg.height, dtype=bool)
            for i in range(num_range_bins - 1):
                point_counts = []
                close, far = r_vals[i], r_vals[i + 1]
                ranges = np.sqrt(pc_data[:, channel, 0] ** 2 + \
                    pc_data[:, channel, 1] ** 2 + \
                    pc_data[:, channel, 2] ** 2)
                range_mask = (ranges > close) & (ranges < far)
                for z in z_vals:
                    z_mask = range_mask & (pc_data[:, channel, 2] < z)
                    point_counts.append(np.count_nonzero(z_mask))
                
                dp = np.diff(point_counts, 1)
                dz = np.diff(z_vals, 1)
                deriv = dp / dz
                deriv_z = 0.5 * (z_vals[:-1] + z_vals[1:])
                ddp = np.diff(dp, 1)
                ddz = np.diff(deriv_z, 1)
                dderiv = ddp / ddz
                dderiv_z = 0.5 * (deriv_z[:-1] + deriv_z[1:])
                
                ground_z = dderiv_z[np.argmin(dderiv)]
                range_mask &= (pc_data[:, channel, 2] > ground_z + 0.02) & (pc_data[:, channel, 2] < self.max_z)
                channel_mask |= range_mask
                plt.plot(z_vals, point_counts)
                plt.axvline(ground_z)
                
            mask[:, channel] |= channel_mask
                #plt.plot(0.5 * (new_z[:-1] + new_z[1:]), dderiv)        
        plt.show()
                




        """
        start = time.perf_counter()
        range_values = np.sqrt(pc_data[:, :, 0] ** 2 + pc_data[:, :, 1] ** 2 + pc_data[:, :, 2] ** 2)
        point_counts = []
        for z in z_vals:
            current_mask = (range_values < self.max_range) & (pc_data[:, :, z_index] < z)
            point_counts.append(np.count_nonzero(current_mask))
        
        plt.plot(z_vals, point_counts)
        plt.show()
        
        mask = (range_values <= self.max_range) & (pc_data[:, :, z_index] <= self.max_z)
        index_list = np.where(mask)
        valid_points = pc_data[index_list[0], index_list[1], :3]
        outlier_indices = self.plane_RANSAC(valid_points, 30, 0.025)
        mask[index_list[0][outlier_indices], index_list[1][outlier_indices]] = False
        """
        
        pc_data[~mask, z_index] = np.nan
        
        # Create a new PointCloud2 message
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = msg.height
        filtered_msg.width = msg.width
        filtered_msg.fields = msg.fields
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = msg.row_step
        filtered_msg.is_dense = msg.is_dense

        # Set the modified array as the data field of the new PointCloud2 message
        filtered_msg.data = pc_data.tobytes(order='C') # Very slow!!!
        end = time.perf_counter()
        filter_time = end - start

        # Publish the modified PointCloud2 message
        self.filtered_pub.publish(filtered_msg)
        self.get_logger().info(f'Conversion time: {"%.3f" % conv_time}\tFiltering time: {"%.3f" % filter_time}')
    
    def plane_RANSAC(self, points: np.array, num_iterations, tol):
        num_points = points.shape[0]
        best_outliers = []
        best_num_outliers = num_points
        
        
        for _ in range(num_iterations):
            sample = random.sample(range(num_points), 3)
            p1, p2, p3 = points[sample[0], :], points[sample[1], :], points[sample[2], :]
            
            v1 = p2 - p1
            v2 = p3 - p1
            
            plane_normal = np.cross(v1, v2)
            plane_normal /= np.linalg.norm(plane_normal)
            distance = np.dot(-plane_normal, p1.T)
            
            outliers = []
            for i, point in enumerate(points):
                dist = abs(np.dot(plane_normal, point) + distance)
                if dist < tol:
                    outliers.append(i)

            num_outliers = len(outliers)

            if num_outliers < best_num_outliers:
                best_outliers = outliers
                best_num_outliers = num_outliers
            
        return best_outliers

def main(args=None):
    rclpy.init(args=args)
        
    node = LidarNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
