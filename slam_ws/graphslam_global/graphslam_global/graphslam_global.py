import numpy as np
from graphslamrs import GraphSLAMSolve
from all_settings.all_settings import GraphSLAMSettings as settings
from all_settings.all_settings import GraphSLAMRSSolverSettings as solver_settings
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
from .utility_functions import compute_timediff, quat_to_euler, compute_delta_velocity, cartesian_to_polar
from .ground_truth_publisher import Ground_Truth_Publisher

from feb_msgs.msg import State, FebPath, Map, Cones
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance


class GraphSLAM_Global(Node):
    def __init__(self):
        print("SLAM INIT")
        # ROS2 INTEGRATIONS
        super().__init__('graphslam_global_node')
        
        self.slam = GraphSLAMSolve(**solver_settings)
        self.publish_to_rviz = settings.publish_to_rviz
        self.local_radius = settings.local_radius
        self.local_vision_delta = settings.local_vision_delta
        self.solve_by_time = settings.solve_by_time
        self.using_wheelspeeds = settings.using_wheelspeeds
        self.imu_direction = settings.forward_imu_direction
        if self.solve_by_time: 
            self.solve_frequency = settings.solve_frequency
        else:
            self.solve_distance = settings.solve_distance

        # Simulator Specific Settings
        self.using_simulator = settings.using_simulator
        if (self.using_simulator):
            self.using_ground_truth_cones = settings.using_ground_truth_cones
            self.using_ground_truth_wheelspeeds = settings.using_ground_truth_wheelspeeds
            self.using_ground_truth_state = settings.bypass_SLAM

        self.statetimestamp = 0.0
        self.currentstate = State()
        self.currentstate.x = 0.0 
        self.currentstate.y = 0.0
        self.currentstate.velocity = 0.0
        self.currentstate.heading = 0.0
        self.currentstate.lap_count = 0
        self.global_map = Map()
        self.local_map = Map()
        self.time = time.time()
        
        # how far into periphery of robot heading on each side to include local cones (robot has tunnel vision if this is small) (radians)
        # for handling new messages during the solve step
        self.finished = False
        self.last_slam_update = np.array([np.Inf, np.Inf])
        # SUBSCRIBERS

        # SIMULATOR SPECIFIC SUBSCRIBERS 
        if (self.using_simulator):
            if (self.using_ground_truth_cones):
                self.cones_sub = self.create_subscription(ConeArrayWithCovariance, '/ground_truth/cones', self.cones_callback, 1)
            else: 
                self.cones_sub = self.create_subscription(ConeArrayWithCovariance, '/fusion/cones', self.cones_callback, 1)   
            if (self.using_wheelspeeds):
                if (self.using_ground_truth_wheelspeeds):
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ground_truth/wheel_speeds', self.wheelspeed_sub_sim, 1)
                else: 
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ros_can/wheel_speeds', self.wheelspeed_sub_sim, 1)
            
            self.state_subby = self.create_subscription(CarState, '/ground_truth/state', self.state_sub, 1,)
        else: 
            self.cones_sub = self.create_subscription(Cones, '/perception_cones', self.cones_callback, 1)
            if (self.using_wheelspeeds):
                self.wheelspeeds = self.create_subscription(Float64, '/odometry/wheelspeeds', self.wheelspeed_sub, 1)

        # IMU subscriber the same for both car and sim
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        # PUBLISHERS
        # Publish the current vehicle's state: X, Y, Velo, Theta
        self.state_pub = self.create_publisher(State, '/slam/state', 1)

        # Publish the current map (GLOBAL_NODE, so this will send the whole map)
        self.global_map_pub = self.create_publisher(Map, '/slam/map/global', 1)
        self.lap_counter_pub = self.create_publisher(Float64, '/slam/lap_counter', 1)

        self.local_map_pub = self.create_publisher(Map, '/slam/map/local', 1)
        self.steering_sub = self.create_subscription(Float64, '/control/steer', self.cmd_callback, 1)

        if (self.publish_to_rviz):
            ##These are for Visuals in the SIM 
            self.cones_vis_pub = self.create_publisher(PointCloud, '/slam/conemap', 1)
            self.positionguess = self.create_publisher(PointCloud, '/slam/guessed_positions', 1)
            self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 1)

        self.orange_y_bounds = []
        self.lap_counter = -1
        self.end_race_pub = self.create_publisher(Bool, '/end_race', 1)
    def cmd_callback(self, cmd: Float64):
        self.currentstate.theta = cmd.data
    def state_sub(self, state: CarState):
        """ This is a callback function for the SIMULATORS ground truth carstate. 
            Currently being used to get the cars position so we can calculate the cones R, theta properly.
        """
        if self.using_ground_truth_state: 
            self.update_state(
                [state.pose.pose.position.x - self.currentstate.x, state.pose.pose.position.y - self.currentstate.y],
                quat_to_euler(state.pose.pose.orientation),
                np.sqrt(state.twist.twist.linear.x**2 + state.twist.twist.linear.y**2)
            )
        return

    def wheelspeed_sub_sim(self, msg: WheelSpeedsStamped):
        self.currentstate.velocity = ((msg.speeds.lb_speed + msg.speeds.rb_speed)/2)*np.pi*0.505/60

    def wheelspeed_sub(self, msg: Float64): 
        self.velocity = msg.data

    def update_state(self, dx: np.array, yaw: float, velocity: float) -> None:
        """
        Function that updates the State.msg variables after a new state has been produced
        Inputs: 
        - dx: change in position [delta_x, delta_y]
        - yaw: new heading (theta)
        - velocity
        Outputs: None
        """
        # All carstates should be float #'s 
        if len(self.orange_y_bounds) > 0:
            if self.orange_y_bounds[0] < self.currentstate.y < self.orange_y_bounds[1]:
                if self.currentstate.x < self.orange_x_pos and self.currentstate.x + dx[0] > self.orange_x_pos:
                    self.lap_counter += 1
                elif self.currentstate.x > self.orange_x_pos and self.currentstate.x + dx[0] < self.orange_x_pos:
                    self.lap_counter -= 1
            if self.lap_counter > 0:
                self.local_map.header.stamp = self.get_clock().now().to_msg()
                map_msg = Map()
                left = np.array(self.slam.get_cones(1))
                right = np.array(self.slam.get_cones(2))
                map_msg.left_cones_x = left[:, 0].flatten().tolist()
                map_msg.left_cones_y = left[:, 1].flatten().tolist()
                map_msg.right_cones_x = right[:, 0].flatten().tolist()
                map_msg.right_cones_y = right[:, 1].flatten().tolist()
                self.global_map_pub.publish(map_msg)
                self.finished = True
            if self.lap_counter > settings.NUM_LAPS:
                msg = Bool()
                msg.data = True
                self.end_race_pub.publish(msg)

        self.currentstate.x += dx[0]
        self.currentstate.y += dx[1]
        self.currentstate.velocity = velocity
        self.currentstate.heading = yaw
        self.currentstate.header.stamp = self.get_clock().now().to_msg()
        msg = Float64()
        msg.data = float(self.lap_counter)
        self.lap_counter_pub.publish(msg)





    def imu_callback(self, imu: Imu) -> None:
        """
        Function that takes in IMU messages and processes GraphSLAM based upon 
        Input: imu (Imu_msg)
        - geometry_msgs/Quarternion orientation
        - float64[9] orientation_covariance
        - geometry_msgs/Vector3 angular_velocity
        - float64[9] angular_velocity_covariance
        - geometry_msgs/Vector3 linear_acceleration
        - float64[9] linear_acceleration_covariance
        """
        if self.using_ground_truth_state:
            return
        # process time
        dt = compute_timediff(self, imu.header)
        if (dt > 1):
            return

        self.currentstate.heading = quat_to_euler(imu.orientation)

        if (self.using_wheelspeeds == False): 
            delta_velocity = compute_delta_velocity(imu.linear_acceleration, dt, self.imu_direction)
            velocity = self.currentstate.velocity + delta_velocity
            self.currentstate.velocity = velocity

        # for now, we assume velocity is in the direction of heading
        # generate dx [change in x, change in y] to add new pose to graph
        dx = self.currentstate.velocity * dt * np.array([math.cos(self.currentstate.heading), math.sin(self.currentstate.heading)])
        
        self.update_state(dx, self.currentstate.heading, self.currentstate.velocity)
        self.state_pub.publish(self.currentstate)

        if self.publish_to_rviz: 
            ## Show the estimated Pose on the Sim        
            pose_msg = PoseStamped()
    
            pose_msg.pose.position.x = self.currentstate.x
            pose_msg.pose.position.y = self.currentstate.y
            pose_msg.pose.position.z = 0*self.currentstate.velocity
            pose_msg.pose.orientation.w = np.cos(self.currentstate.heading/2)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = np.sin(self.currentstate.heading/2)
    
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()

            self.pose_pub.publish(pose_msg)



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
            for cone in cones.big_orange_cones:
                r, theta = cartesian_to_polar([0.0, 0.0], (cone.point.x, cone.point.y))
                cone_matrix[0].append(r)
                cone_matrix[1].append(theta)
                cone_matrix[2].append(0)

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

        
        distance_solve_condition = np.linalg.norm(self.last_slam_update-np.array([self.currentstate.x, self.currentstate.y])) > 1.0
        time_solve_condition = time.time() - self.time > 0.3
        # Check depending on if ready to solve based on whether you're solving by time or by condition
        ready_to_solve = (self.solve_by_time and time_solve_condition) or (not self.solve_by_time and distance_solve_condition)

        if self.finished:
            colors = np.array([2]*len(cones.blue_cones) + [1]*len(cones.yellow_cones)).astype(np.uint8)
            cones = np.array([[i.point.x for i in cones.blue_cones] + [i.point.x for i in cones.yellow_cones],
                           [i.point.y for i in cones.blue_cones] + [i.point.y for i in cones.yellow_cones]])
            rot = lambda x: np.array([[np.cos(x), -np.sin(x)], [np.sin(x), np.cos(x)]])
            cones = rot(-self.currentstate.heading)@cones
            res = self.slam.pose_from_data_association([self.currentstate.x, self.currentstate.y, 0.0], cones.T, colors)
            self.update_state([res[0] - self.currentstate.x, res[1]- self.currentstate.y], self.currentstate.heading + res[2], self.currentstate.velocity)
            # self.currentstate.x = res[0]
            # self.currentstate.y = res[1]
            # self.currentstate.heading += res[2]
            # print(res)
            return
        if ready_to_solve:
            # last_slam_update initialized to infinity, so set current state x,y to 0 in the case. otherwise, update graph with relative position from last update graph
            self.slam.update_graph(
                np.array([self.currentstate.x, self.currentstate.y])-self.last_slam_update if self.last_slam_update[0]<999999999.0 else np.array([0.0, 0.0]), 
                                   cartesian_cones[:, :2], 
                                   cartesian_cones[:, 2].flatten().astype(np.uint8).tolist()) # old pre-ros threading
            # print(cartesian_cones.T.shape)
            self.last_slam_update = np.array([self.currentstate.x, self.currentstate.y])
            self.time = time.time()
            self.slam.solve_graph()
        else:
            return
        #if (abs(self.time - time.time()) > 0.3): #NOTE self.time - time.time() should be negative
        # print("MADE IT HERE")
        # last_slam_update initialized to infinity, so set current state x,y to 0 in the case. otherwise, update graph with relative position from last update graph
        self.slam.update_graph(np.array([self.currentstate.x, self.currentstate.y])-self.last_slam_update if self.last_slam_update[0]<999999999.0 else np.array([0.0, 0.0]), 
                                cartesian_cones[:, :2], 
                                cartesian_cones[:, 2].flatten().astype(np.uint8).tolist()) # old pre-ros threading
        # print(cartesian_cones.T.shape)
        self.last_slam_update = np.array([self.currentstate.x, self.currentstate.y])
        self.time = time.time()
        self.slam.solve_graph()
        # print("UPDATING STATE")
        if len(self.orange_y_bounds)==0:
            orange_cones = np.array(self.slam.get_cones(0)) # orange cones
            if (len(orange_cones.flatten())>2) and ((max_y:=np.max(orange_cones[:, 1])) - (min_y:=np.min(orange_cones[:, 1])) > 2):
                self.orange_y_bounds = [min_y, max_y]
                self.orange_x_pos = np.average(orange_cones[:, 0])

        
        x_guess, lm_guess = np.array(self.slam.get_positions()), np.array(self.slam.get_cones())

        # Publish Running Estimate of Car Positions & Cone Positions in respective messages: positionguess & cone_vis_pub

        pos = np.array(x_guess[-1]).flatten()
        self.update_state([pos[0] - self.currentstate.x, pos[1]-self.currentstate.y], self.currentstate.heading, self.currentstate.velocity)
        self.currentstate.x = pos[0]
        self.currentstate.y = pos[1]

        #x and lm guess come out as lists, so change to numpy arrays
        x_guess = np.array(x_guess)
        lm_guess = np.array(lm_guess)
        

        # blue_array = np.array([2 for i in range(len(lm_guess[:,2]))])
        # left_cones = lm_guess[np.round(lm_guess[:,2]) == 2][:,:2] # blue
        # right_cones = lm_guess[np.round(lm_guess[:,2]) == 1][:,:2] # yellow
        left_cones = np.array(self.slam.get_cones(2))
        right_cones = np.array(self.slam.get_cones(1))
        # print(left_cones)
        # print(right_cones)
        if left_cones.shape==(0,): left_cones = left_cones.reshape((0, 2))
        if right_cones.shape==(0,): right_cones = right_cones.reshape((0, 2))
        # print(self.slam.color.shape, self.slam.lhat.shape, left_cones.shape, right_cones.shape, lm_guess, cartesian_cones)

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
            # blue_array = np.array([2 for i in range(len(lm_guess[:,2]))])
            # left_cones = lm_guess[np.round(lm_guess[:,2]) == 2][:,:2] # blue
            # right_cones = lm_guess[np.round(lm_guess[:,2]) == 1][:,:2] # yellow
            # print(self.slam.color.shape, self.slam.lhat.shape, left_cones.shape, right_cones.shape, lm_guess, cartesian_cones)
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

        ret_localcones_left = []
        ret_localcones_right = []

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
        
