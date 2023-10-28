import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import numpy as np
import time

import auto_calibration.utils as ut

class LidarBoardDetector(Node):
    def __init__(self):
        super().__init__('lidar_board_detector')
        
        # Publishers
        self.pc_publisher = self.create_publisher(PointCloud2, '/board_points', 10)
        self.cp_publisher = self.create_publisher(PointCloud2, '/clicked', 10)
        
        # Subscribers
        self.pc_subscriber = self.create_subscription(PointCloud2, '/rslidar_points', self.callback, 10)
        self.cp_subscriber = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        
        # Global variables
        self.current_pc = None
        self.board_radius = 0.5
        self.plane_ransac_tol = 0.025
        self.plane_ransac_iter = 20

        
    def callback(self, msg: PointCloud2):
        # Just save the current pointcloud
        self.current_pc = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
    
    
    def clicked_point_callback(self, msg: PointStamped):
        # Timekeeping
        filtering_time = float
        ransac_time = float
        publishing_time = float
        
        clicked_x, clicked_y, clicked_z = msg.point.x, msg.point.y, msg.point.z
        # For publishing stuff 
        header = Header()
        header.frame_id = "rslidar"
        cp_to_publish = pc2.create_cloud_xyz32(header, [(clicked_x, clicked_y, clicked_z)])
        self.cp_publisher.publish(cp_to_publish)
        
        # Only keep points within the tolerance of the clicked point
        start = time.perf_counter()
        filtered_points = []
        for p in range(len(self.current_pc)):
            px, py, pz = self.current_pc[p][0], self.current_pc[p][1], self.current_pc[p][2]
            dist_sq = (px - clicked_x)**2 + (py - clicked_y)**2 + (pz - clicked_z)**2
            
            if dist_sq <= self.board_radius**2:
                filtered_points.append([px, py, pz])
        end = time.perf_counter()
        filtering_time = end - start
        
        # RANSAC to only keep board points
        start = time.perf_counter()
        board_points = ut.plane_RANSAC(np.array(filtered_points), self.plane_ransac_iter, self.plane_ransac_tol)
        end = time.perf_counter()
        ransac_time = end - start
        
        # Convert  to list of voids
        start = time.perf_counter()
        list_to_publish = []
        for i in range(board_points.shape[0]):
            row = board_points[i, :]
            point = (row[0], row[1], row[2])
            list_to_publish.append(point)
        
        # Create and publish cloud
        cloud = pc2.create_cloud_xyz32(header, list_to_publish)
        self.pc_publisher.publish(cloud)
        end = time.perf_counter()
        publishing_time = end - start
        
        self.get_logger().info(f"\nFiltering time:\t\t{'%.5f' % filtering_time} s.\n\
RANSAC time:\t\t{'%.5f' % ransac_time} s.\n\
Publishing time:\t{'%.5f' % publishing_time} s.\n")
        
    
def main(args=None):
    rclpy.init(args=args)
        
    node = LidarBoardDetector()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
