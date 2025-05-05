import numpy as np
from .GraphSLAMSolve import GraphSLAMSolve
from all_settings.all_settings import GraphSLAMSolverSettings as solver_settings
from all_settings.all_settings import GraphSLAMSettings as settings
from time import perf_counter
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header, Bool
from sensor_msgs.msg import Imu
from eufs_msgs.msg import WheelSpeedsStamped
from eufs_msgs.msg import CarState
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Pose, Point, Point32
from sensor_msgs.msg import PointCloud
from .utility_functions import compute_timediff, quat_to_euler, compute_delta_velocity, cartesian_to_polar, rotate_and_translate_cones
from .ground_truth_publisher import Ground_Truth_Publisher
from .mwekf_backend import MWEKF_Backend

from feb_msgs.msg import State, FebPath, Map, Cones, ConesCartesian
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from ackermann_msgs.msg import AckermannDriveStamped


class GraphSLAM_MWEKF(Node):
    def __init__(self):
        # ROS2 INTEGRATIONS
        super().__init__('graphslam_global_node')
        
        self.slam = GraphSLAMSolve(**solver_settings)
        self.mwekf = MWEKF_Backend(self, np.array([0., 0., 0., 0.]), 2)
        # SUBSCRIBERS
        # SIMULATOR SPECIFIC SUBSCRIBERS 
        if (settings.using_simulator):
            if (settings.using_wheelspeeds):
                if (settings.using_ground_truth_wheelspeeds):
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ground_truth/wheel_speeds', self.wheelspeed_sub_sim, 1)
                else: 
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ros_can/wheel_speeds', 1)
            #self.state_subby = self.create_subscription(CarState, '/ground_truth/state', self.state_sub, 1,)
            self.camera_sub = self.create_subscription(ConesCartesian, '/camera/cones', self.camera_callback, 1)
        else: 
            self.wheelspeeds = self.create_subscription(Float64, '/odometry/wheelspeeds', self.wheelspeed_sub, 1)
            self.realsense_d435i_sub = self.create_subscription(ConesCartesian, '/realsense/d435i/cones', self.d435i_cones_callback, 1)
            self.realsense_d435_sub = self.create_subscription(ConesCartesian, '/realsense/d435/cones', self.d435_cones_callback, 1)


        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)
        self.cones_lidar_sub = self.create_subscription(ConesCartesian, '/lidar/cones', self.lidar_callback, 1)
        self.mpc_sub = self.create_subscription(AckermannDriveStamped, '/cmd', self.mpc_callback, 1)
        # PUBLISHERS
        self.state_pub = self.create_publisher(State, '/slam/state', 1)
        self.global_map_pub = self.create_publisher(Map, '/slam/map/global', 1)
        self.local_map_pub = self.create_publisher(Map, '/slam/map/local', 1)

        if (settings.publish_to_rviz):
            self.cones_vis_pub = self.create_publisher(PointCloud, '/slam/conemap', 1)
            self.positionguess = self.create_publisher(PointCloud, '/slam/guessed_positions', 1)
            self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 1)

        self.get_logger().info("Initialized le MWEKF")


    def mpc_callback(self, msg: AckermannDriveStamped):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        self.mwekf.last_u[:] = [msg.drive.acceleration, msg.drive.steering_angle, sec + nsec * 1e-9]
        print("CMD: ", msg.drive.acceleration, msg.drive.steering_angle)

    def wheelspeed_sub_sim(self, msg: WheelSpeedsStamped):
        self.mwekf.update((((msg.speeds.lb_speed + msg.speeds.rb_speed)/2)*np.pi*0.505/60), 3)

    def wheelspeed_sub(self, msg: Float64): 
        self.mwekf.update(msg.data, 3)

    def imu_callback(self, imu: Imu) -> None:
        if settings.using_simulator:
            self.mwekf.update(quat_to_euler(imu.orientation), 2)
        else: 
            forward = settings.imu_foward_direction
        pass
    
    def d435i_cones_callback(self, msg: ConesCartesian):
        self.cones_callback(rotate_and_translate_cones(msg, 'd435i'))

    def d435_cones_callback(self, msg: ConesCartesian):
        self.camera_callback(rotate_and_translate_cones(msg, 'd435'))

    def camera_callback(self, msg: ConesCartesian):
        return

    def lidar_callback(self, msg: ConesCartesian):
        return

    def create_mwekf(self):
        pass

    def load_mwekf_to_slam(self):
        pass


    def cones_callback(self, cones) -> None:
        """
        Function that takes the list of cones, updates and solves the graph
    
        """
        if self.using_simulator: 
            cone_matrix = [[], [], []]
            for cone in cones.blue_cones:
                r, theta = cartesian_to_polar([0.0, 0.0], (cone.point.x, cone.point.y))
                cone_matrix[0].append(r)
                cone_matrix[1].append(theta)
                cone_matrix[2].append(2)
            for cone in cones.yellow_cones:
                r, theta = cartesian_to_polar([0.0, 0.0], (cone.point.x, cone.point.y))
                cone_matrix[0].append(r)
                cone_matrix[1].append(theta)
                cone_matrix[2].append(1)
 

            cone_matrix = np.array(cone_matrix).T
            cone_dx = cone_matrix[:,0] * np.cos(cone_matrix[:,1]+self.currentstate.heading) # r * cos(theta) element wise
            cone_dy = cone_matrix[:,0] * np.sin(cone_matrix[:,1]+self.currentstate.heading) # r * sin(theta) element_wise
            cartesian_cones = np.vstack((cone_dx, cone_dy, cone_matrix[:,2])).T # n x 3 array of n cones and dx, dy, color   -- input for update_graph

        else: 
            cone_matrix = [[], [], []]
            array_len = len(cones.r)
            for idx in range(array_len): 
                if cones.r[idx] >  self.local_radius: 
                    pass
                else:
                    cone_matrix[0].append(cones.r[idx])
                    cone_matrix[1].append(cones.theta[idx])
                    cone_matrix[2].append(cones.color[idx])
        
            cone_matrix = np.array(cone_matrix).T
            cone_dx = cone_matrix[:,0] * np.cos(cone_matrix[:,1]+self.currentstate.heading)# r * cos(theta) element wise
            cone_dy = cone_matrix[:,0] * np.sin(cone_matrix[:,1]+self.currentstate.heading) # r * sin(theta) element_wise
            cartesian_cones = np.vstack((cone_dx, cone_dy, cone_matrix[:,2])).T # n x 3 array of n cones and dx, dy, color   -- input for update_graph

        # Dummy function for now, need to update graph and solve graph on each timestep
        if self.finished:
            return
        
        distance_solve_condition = np.linalg.norm(self.last_slam_update-np.array([self.currentstate.x, self.currentstate.y])) > 1.0
        time_solve_condition = time.time() - self.time > 0.3
        # Check depending on if ready to solve based on whether you're solving by time or by condition
        ready_to_solve = (self.solve_by_time and time_solve_condition) or (not self.solve_by_time and distance_solve_condition)

        if ready_to_solve:
            # last_slam_update initialized to infinity, so set current state x,y to 0 in the case. otherwise, update graph with relative position from last update graph
            self.slam.update_graph(
                np.array([self.currentstate.x, self.currentstate.y])-self.last_slam_update if self.last_slam_update[0]<999999999.0 else np.array([0.0, 0.0]), 
                                   cartesian_cones[:, :2], 
                                   cartesian_cones[:, 2].flatten())
            self.last_slam_update = np.array([self.currentstate.x, self.currentstate.y])
            self.time = time.time()
            self.slam.solve_graph()
        else:
            return
    
        
        x_guess, lm_guess = self.slam.xhat, np.hstack((self.slam.lhat, self.slam.color[:, np.newaxis]))

        # Publish Running Estimate of Car Positions & Cone Positions in respective messages: positionguess & cone_vis_pub

        pos = np.array(x_guess[-1]).flatten()
        self.currentstate.x = pos[0]
        self.currentstate.y = pos[1]

        #x and lm guess come out as lists, so change to numpy arrays
        x_guess = np.array(x_guess)
        lm_guess = np.array(lm_guess)
        
        left_cones = lm_guess[np.round(lm_guess[:,2]) == 2][:,:2] # blue
        right_cones = lm_guess[np.round(lm_guess[:,2]) == 1][:,:2] # yellow

        #filter local conesfor sim & publihs in local_map_pub

        local_left, local_right = self.localCones(self.local_radius, left_cones, right_cones)
        if (len(np.array(local_left)) == 0 or len(np.array(local_right)) == 0):
            return
        #update map message with new map data 

        self.local_map.left_cones_x = np.array(local_left)[:,0].tolist()
        self.local_map.left_cones_y = np.array(local_left)[:,1].tolist()
        self.local_map.right_cones_x = np.array(local_right)[:,0].tolist()
        self.local_map.right_cones_y = np.array(local_right)[:,1].tolist()

        #update message header
        #self.local_map.header.seq = self.seq
        self.local_map.header.stamp = self.get_clock().now().to_msg()
        self.local_map.header.frame_id = "map"

        self.local_map_pub.publish(self.local_map)

        if self.publish_to_rviz:
            # Publish SLAM Position visual
            position_guess = PointCloud()
            positions = []
            for pos in x_guess: 
                positions.append(Point32())
                positions[-1].x = pos[0]
                positions[-1].y = pos[1]
                positions[-1].z = 0.0
            position_guess.points = positions
            position_guess.header.frame_id = "map"
            position_guess.header.stamp = self.get_clock().now().to_msg()
            self.positionguess.publish(position_guess)   

            # Publish SLAM Map visual
            blue_array = np.array([2 for i in range(len(lm_guess[:,2]))])
            left_cones = lm_guess[np.round(lm_guess[:,2]) == 2][:,:2] # blue
            right_cones = lm_guess[np.round(lm_guess[:,2]) == 1][:,:2] # yellow
            print(self.slam.color.shape, self.slam.lhat.shape, left_cones.shape, right_cones.shape, lm_guess, cartesian_cones)
            total_cones = np.vstack((left_cones,right_cones))
            cones_msg = PointCloud()
            cones_to_send = []
            for cone in lm_guess: 
                cones_to_send.append(Point32())
                cones_to_send[-1].x = cone[0]
                cones_to_send[-1].y = cone[1]
                cones_to_send[-1].z = 0.0
            cones_msg.points = cones_to_send
            cones_msg.header.frame_id = "map"
            cones_msg.header.stamp = self.get_clock().now().to_msg()
            self.cones_vis_pub.publish(cones_msg)


    # publishes all cones within given radius
    def localCones(self, radius, left, right):
        """
        Find cones within a given radius around the car's current position.

        Parameters:
            radius (float): The radius within which to search for cones.
            left (list): List of left cones.
            right (list): List of right cones.

        Returns:
            tuple: A tuple containing lists of left and right cones found within the radius.
        """
        left = np.array(left)
        right = np.array(right)
        curpos = np.array([self.currentstate.x, self.currentstate.y])
        heading = self.currentstate.heading

        # Cones within radius
        close_left = left[((left - curpos) ** 2).sum(axis=1) < radius * radius]
        close_right = right[((right - curpos) ** 2).sum(axis=1) < radius * radius]


        # Concatenate close_left and close_right
        close = np.concatenate((close_left, close_right), axis=0)
        left_len = len(close_left)

        # Calculate delta vectors
        # take positions of the cones relative to the car - and then rotates it by heading to get true location
        delta_vecs = close - curpos
        delta_vecs = (np.array([[np.cos(-heading), -np.sin(-heading)],
                                 [np.sin(-heading), np.cos(-heading)]])@(delta_vecs.T)).T

        #take cones in front of the car
        mask = delta_vecs[:, 0]>0

        to_plot = close[mask]
        if self.publish_to_rviz:
            cones_msg = PointCloud()
            cones_to_send = []
            for cone in to_plot: 
                cones_to_send.append(Point32())
                cones_to_send[-1].x = cone[0]
                cones_to_send[-1].y = cone[1]
                cones_to_send[-1].z = 0.0
            cones_msg.points = cones_to_send
            cones_msg.header.frame_id = "map"
            cones_msg.header.stamp = self.get_clock().now().to_msg()
            self.cones_vis_pub.publish(cones_msg)



        return close[:left_len][mask[:left_len]], close[left_len:][mask[left_len:]]
        