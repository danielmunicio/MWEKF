import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, String
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import sys

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


class CameraProjection(Node):
    def __init__(self):
        super().__init__('camera_projection')
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/color/image_raw', self.camera_callback, 10)
        self.point_cloud_sub = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_callback, 10)

        # Publishers
        self.processed_img_publisher = self.create_publisher(Image, '/camera_projection', 10)
        
        # Global variables
        self.camera_matrix = np.array([
            [914.969275, 0., 654.163992],
            [0., 914.443972, 371.069725],
            [0., 0., 1.]
        ])
        
        self.dist_coeffs = np.array([0.119090, -0.250076, 0.005887, 0.002350, 0.])
        self.bridge = CvBridge()
        self.received_point_cloud = False
        self.point_cloud = None
        self.processed_img_published = False
    
    
    def lidar_callback(self, msg: PointCloud2):
        if self.received_point_cloud:
            return
        self.received_point_cloud = True
        
        # Save current pointcloud
        self.point_cloud = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        
    
    
    def camera_callback(self, msg: Image):
        if not self.received_point_cloud or self.processed_img_published:
            return

        
        # Calculate ranges of points
        num_points = len(self.point_cloud)
        points_3d = np.zeros((num_points, 3))
        ranges_sq = np.zeros((1, num_points))
        max_range = 0.
        min_range = sys.float_info.max
        start = time.perf_counter()
        for i in range(num_points):
            x, y, z = self.point_cloud[i][0], self.point_cloud[i][1], self.point_cloud[i][2]
            # Extrinsics
            x, y, z = -y, -z + .15, x
            r2 = x**2 + y**2 + z**2
            if r2 > max_range:
                max_range = r2
            if r2 < min_range:
                min_range = r2
            points_3d[i, :] = [x, y, z]
            ranges_sq[:, i] = r2
        points_2d, _ = cv2.projectPoints(points_3d, (0,0,0), (0,0,0), self.camera_matrix, self.dist_coeffs)
        points_2d = points_2d[:, 0].astype(int)
        end = time.perf_counter()
        print(f"Parsing time: {end - start} s.")
        
        # Convert the Image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        
        # Undistort the image
        undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
        
        # Draw the projected point on the undistorted image
        start = time.perf_counter()
        for i in range(num_points):
            # Color points accordingly
            ratio = 2 * (ranges_sq[0, i] - min_range) / (max_range - min_range)
            b = int(max(0, 255*(1 - ratio)))
            r = int(max(0, 255*(ratio - 1)))
            g = 255 - b - r
            
            # Draw the projected points
            cv2.circle(undistorted_image, (points_2d[i, 0], points_2d[i, 1]), 1, (r, g, b), -1)
        end = time.perf_counter()
        print(f"Drawing time: {end - start} s.")

        # Convert the modified image back to an Image message
        modified_msg = self.bridge.cv2_to_imgmsg(undistorted_image, 'bgr8')

        # Publish the modified image
        self.processed_img_publisher.publish(modified_msg)
        self.processed_img_published = True
    
def main(args=None):
    rclpy.init(args=args)
        
    # node = LidarBoardDetector()
    node = CameraProjection()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
