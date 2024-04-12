import numpy as np
from numpy.linalg import *
# import matplotlib.pyplot as plt
# import csv
# import track_generator_leonid.track_main as trackmain
# import track_generator_leonid.settings as settings
from .graphslam_colorsolve import GraphSLAM
from time import perf_counter
import math
# from numpy.random import random, randn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header, Bool
from sensor_msgs.msg import Imu
from eufs_msgs.msg import WheelSpeedsStamped
from eufs_msgs.msg import CarState
from geometry_msgs.msg import Quaternion, Vector3

import matplotlib.pyplot as plt 

# import sys
# import os 
# print(sys.path)

# current_dir = os.getcwd() #feb_system_integration/slam_ws/src/graphslam_global/graphslam_global
# root_dir = os.path.dirname( #feb_system_integration
#     os.path.dirname( #feb_system_integration/slam_ws/
#         os.path.dirname( #feb_system_integration/slam_ws/src
#             os.path.dirname(current_dir)))) #feb_system_integration/slam_ws/src/graphslam_global

# msg_path = 'comm_ws/src/feb_msgs/msg'
# abs_msg_path = os.path.join(current_dir, msg_path)
# print(abs_msg_path)
#sys.path.append(abs_msg_path)



#file_msgdir_path = os.path.join(abs_msg_path, 'example2.txt')
#if not os.path.exists(abs_msg_path):
#    os.makedirs(abs_msg_path)
#with open(file_msgdir_path, 'w') as file:
#    file.write("example file")
#sys.path.append(file_msgdir_path)

#import State.msg, FebPath, Map, Cones

from feb_msgs.msg import State, FebPath, Map
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance

class GraphSLAM_Global(Node):
    def __init__(self):

        # ROS2 INTEGRATIONS

        super().__init__('graphslam_global_node')

        # SUBSCRIBERS

        # Handle IMU messages for vehicle state
        # self.imu_sub = self.create_subscription(
        #     Imu,
        #     '/imu',
        #     self.imu_callback,
        #     1
        # )

        # Handle new cone readings from perception
        self.cones_sub = self.create_subscription(
            ConeArrayWithCovariance,
            '/ground_truth/cones', 
            self.cones_callback,
            1
        )

        # Once path is finished, turn this node callbacks funcs off
        self.finish_sub = self.create_subscription(
            Bool,
            '/path/finished',
            self.finish_callback,
            1
        )

        # PUBLISHERS
        
        # Publish the current vehicle's state: X, Y, Velo, Theta
        self.state_pub = self.create_publisher(
            State,
            '/slam/state',
            1
        )

        # Publish the current map (GLOBAL_NODE, so this will send the whole map)
        self.global_map_pub = self.create_publisher(
            Map, 
            '/slam/map/global',
            1
        )

        self.local_map_pub = self.create_publisher(
            Map, 
            '/slam/map/local',
            1
        )

        self.wheelspeeds = self.create_subscription(
            WheelSpeedsStamped,
            '/ground_truth/wheel_speeds',
            self.wheelspeed_sub,
            1
        )

        self.state_subby = self.create_subscription(
            CarState,
            '/ground_truth/state',
            self.state_sub,
            1,
        )

        # SLAM Initialization

        # Initializes a new instance of graphslam from the graphslam
        # Data Association Threshold is to be tweaked
        self.slam = GraphSLAM(solver_type='qp', landmarkTolerance=4)
        
        # used to calculate the state of the vehicle
        self.statetimestamp = 0.0
        self.currentstate = State()
        self.currentstate.carstate[0] = 0 
        self.currentstate.carstate[1] = 0
        self.currentstate.carstate[2] = 0
        self.currentstate.carstate[3] = 0
        self.state_seq = 0
        self.cone_seq = 0

        self.global_map = Map()
        self.local_map = Map()

        # radius for which to include local cones ---#UPDATE, perhaps from mpc message
        self.local_radius = 100000 
        
        # how far into periphery of robot heading on each side to include local cones (robot has tunnel vision if this is small) (radians)
        self.local_vision_delta = np.pi/2 

        # for handling new messages during the solve step
        self.solving = False

        self.finished = False
        self.usefastslam = False

    def state_sub(self, msg: CarState):
        self.currentstate.carstate[0] = msg.pose.pose.position.x
        self.currentstate.carstate[1] = msg.pose.pose.position.y
        self.currentstate.carstate[3] = self.quat_to_euler(msg.pose.pose.orientation)[-1]
    
    def wheelspeed_sub(self, msg: WheelSpeedsStamped):
        self.currentstate.carstate[2] = ((msg.speeds.lb_speed + msg.speeds.rb_speed)/2)*np.pi*0.505/60

    def finish_callback(self, msg: Bool) -> None:
        if self.usefastslam:
            self.finished = bool(msg.data)

    """
    Function that takes in message header and computes difference in time from last state msg
    Input: Header (std_msg/Header)
    - uint32 seq
    - time stamp
    - string frame_id
    Output: timediff: float
    """
    def compute_timediff(self, header: Header) -> float:
        newtime = header.stamp.sec + 1e-9 * header.stamp.nanosec
        timediff = newtime - self.statetimestamp
        self.statetimestamp = newtime

        return timediff
    
    """
    Function that takes in quaternion and converts to Eulerian angles
    Input: Quat (Quaternion)
    - float x
    - float y
    - float z
    - float w
    Output: roll, pitch, yaw
    #NOTE: roll and pitch are irrelevant as of now, we only care about heading angle (pitch)
    """
    def quat_to_euler(self, quat: Quaternion) -> tuple[float, float, float]:
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * x))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return roll, pitch, yaw

    """
    Function that takes in linear acceleration and dt and outputs velocity (linear)
    Input:
    - linear_acceleration: from imu message
    - dt: calculated at each time step
    Output:
    - linear_velocity
    #NOTE: we assume linear acceleration is in the car's frame. This means
            x is in the longitudinal direction (positive = forwards)
            y is in the lateral direction (positive = to the right)
            depending on how IMU is providing this data, change accordingly
    """
    def compute_velocity(self, acc: Vector3, dt: float) -> float:
        # longitudinal_acc = np.linalg.norm([acc.x, acc.y])
        longitudinal_acc = -acc.x
        # lateral_acc = acc.y # May be needed in the future if  
        #                       straightline model is not accurate enough

        linear_velocity = longitudinal_acc * dt
        return linear_velocity

    """
    Function that updates the State.msg variables after a new state has been produced
    Inputs: 
    - dx: change in position [delta_x, delta_y]
    - yaw: new heading (theta)
    - velocity
    Outputs: None
    """
    def update_state(self, dx: np.array, yaw: float, velocity: float) -> None:
        # x = Float64()
        # x.data = dx[0]
        # y = Float64()
        # y.data = dx[1]

        #This should be deleted 
        v = Float64()
        v.data = velocity
        heading = Float64()
        heading.data = yaw
        #Probably 

        # All carstates should be float #'s 
        self.currentstate.carstate[0] += dx[0]
        self.currentstate.carstate[1] += dx[1]
        self.currentstate.carstate[2] = velocity
        self.currentstate.carstate[3] = yaw
        self.state_seq += 1
        #self.currentstate.header.seq = self.state_seq
        self.currentstate.header.stamp = self.get_clock().now().to_msg()
        self.currentstate.header.frame_id = "rslidar"


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

    def imu_callback(self, imu: Imu) -> None:

        times = [perf_counter()]
        if self.finished:
            return
        # process time
        dt = self.compute_timediff(imu.header)
        if (dt > 1):
            return

        times.append(perf_counter())
        # generate current heading
        roll, pitch, yaw = self.quat_to_euler(imu.orientation)
        times.append(perf_counter())
        # generate current velocity
        delta_velocity = self.compute_velocity(imu.linear_acceleration, dt)
        velocity = self.currentstate.carstate[2] + delta_velocity
        self.currentstate.carstate[2] = velocity
        times.append(perf_counter())
        # for now, we assume velocity is in the direction of heading
        # generate dx [change in x, change in y] to add new pose to graph
        dx = velocity * dt * np.array([math.cos(yaw), math.sin(yaw)])
        times.append(perf_counter())
        # add new position node to graph
        self.slam.update_position(dx)
        times.append(perf_counter())
        #self.slam.update_backlog_imu(dx)
        # update state msg
        self.update_state(dx, yaw, velocity)
        times.append(perf_counter())
        self.state_pub.publish(self.currentstate)
        times.append(perf_counter())

        with open("sim_data.txt", "a") as f:
            print("----------------------------------------------------------", file=f)
            print(f"FROM THE IMU: current x acceleration: {imu.linear_acceleration.x} \n dt: {dt}", file = f)
            print(f"current x position: {self.currentstate.carstate[0]} \n current y position: {self.currentstate.carstate[1]} \n current velocity: {self.currentstate.carstate[2]} \n current yaw: {self.currentstate.carstate[3]}", file=f)
            print("-----------------------------------------------------------")
            print(file=f)
        # print(f"TIME TAKEN FOR IMU CALLBACK: {times}")

    def cartesian_to_polar(self, car_state, cone):
        p_x = cone[0] - car_state[0]
        p_y = cone[1] - car_state[1]
        r = math.sqrt(p_x**2 + p_y**2)
        if (p_x == 0):
            angle = math.asin(p_y/r)
        else:
            angle = math.atan(p_y / p_x)
        if p_x < 0:
            angle = angle + math.pi
        return r, angle
    
    """
    Function that takes the list of cones, updates and solves the graph
    
    """
    def cones_callback(self, cones: ConeArrayWithCovariance) -> None: # abt todo: we have had cones as a placeholder message structure yet to be defined (cones.r, cones.theta, cones.color) for now
        self.local_map.left_cones_x = [cone.point.x for cone in cones.blue_cones]
        self.local_map.left_cones_y = [cone.point.y for cone in cones.blue_cones]
        self.local_map.right_cones_x = [cone.point.x for cone in cones.yellow_cones]
        self.local_map.right_cones_y = [cone.point.y for cone in cones.yellow_cones]
        self.local_map_pub.publish(self.local_map)

        # with open("sim_data.txt", "a") as f:
        #     print("From SLAM cones map:", self.local_map, file=f)
        #     print(file=f)

    
        # x_s = [cone.point.x for cone in cones.blue_cones] + [cone.point.x for cone in cones.yellow_cones]
        # y_s = [cone.point.y for cone in cones.blue_cones] + [cone.point.y for cone in cones.yellow_cones]

        # plt.scatter(x_s, y_s)
        # plt.show()
        return


        # Dummy function for now, need to update graph and solve graph on each timestep
        if self.finished:
            return
        
        #input cone list & dummy dx since we are already doing that in update_graph with imu data
        # cone_matrix = np.hstack(Cones.r, Cones.theta, Cones.color)
        cone_matrix = [[], [], []]
        for cone in cones.blue_cones:
            r, theta = self.cartesian_to_polar(self.currentstate.carstate[:2], (cone.point.x, cone.point.y))
            cone_matrix[0].append(r)
            cone_matrix[1].append(theta)
            cone_matrix[2].append(2)
        for cone in cones.yellow_cones:
            r, theta = self.cartesian_to_polar(self.currentstate.carstate[:2], (cone.point.x, cone.point.y))
            cone_matrix[0].append(r)
            cone_matrix[1].append(theta)
            cone_matrix[2].append(1)

        cone_matrix = np.array(cone_matrix).T
        
        # process all new cone messages separately while one thread is solving slam        
        
        #lock
        #self.slam.update_backlog_perception(cone_matrix)
        #if (self.solving):
        #    return

        self.slam.update_graph_color(cone_matrix) # old pre-ros threading
        
        #self.slam.update_graph_color(perception_backlog_imu, perception_backlog_cones)
        #self.perception_backlog_cones = []
        #self.perception_backlog_imu = []
        #self.slam.update_graph_block()
        #x_guess, lm_guess = self.solveGraphSlamLock()
        x_guess, lm_guess = self.slam.solve_graph()

        #x and lm guess come out as lists, so change to numpy arrays
        x_guess = np.array(x_guess)
        lm_guess = np.array(lm_guess)

        # print('x_guess: ')
        # print(x_guess)
        # print('___________________________')
        # print('lm_guess: ')
        # print(lm_guess)
        # print('___________________________')

        blue_array = np.array([2 for i in range(len(lm_guess[:,2]))])
        # yellow_array = np.array([ for i in range(len(lm_guess[:,2]))])

        # left_cones = lm_guess[lm_guess[:,2] == np.isclose(lm_guess[:,2], 2, rtol = 0.0001, atol = 0.0001)][:,:2] # blue
        #print((lm_guess[:, 2])[0].type())
        #print(type(lm_guess[:, 2]))
        left_cones = lm_guess[np.round(lm_guess[:,2]) == 2][:,:2] # blue
        right_cones = lm_guess[np.round(lm_guess[:,2]) == 1][:,:2] # yellow
        #left_cones = lm_guess[lm_guess[:,2] == 0][:,:2] # orange

        # print('left_cones: ')
        # print(left_cones)
        # print('___________________________')
        # print('right_cones: ')
        # print(right_cones)
        # print('_____________________')

        # #update map message with new map data 
        # self.global_map.left_cones_x = list(left_cones[:,0])
        # self.global_map.left_cones_y = list(left_cones[:,1]) 
        # self.global_map.right_cones_x = list(right_cones[:,0]) 
        # self.global_map.right_cones_y = list(right_cones[:,1]) 

        # #update message header
        # self.cone_seq += 1
        # #self.global_map.header.seq = self.seq
        # self.global_map.header.stamp = self.get_clock().now().to_msg()
        # self.global_map.header.frame_id = "rslidar"

        # self.global_map_pub.publish(self.global_map)
        # print('len of left and right cones:')
        # print(len(left_cones))
        # print(len(right_cones))

        local_left, local_right = self.localCones(self.local_radius, left_cones, right_cones)
        # local_left, local_right = left_cones, right_cones

        # print('len of local left and local right cones:')
        # print(len(local_left))
        # print(len(local_right))
        # print('local left: ')
        # print(local_left)
        # print('local right: ')
        # print(local_right)
        #update map message with new map data 
        with open("out.txt", 'a') as f: 
            print(local_left, file=f)

        self.local_map.left_cones_x = np.array(local_left)[:,0].tolist()
        self.local_map.left_cones_y = np.array(local_left)[:,1].tolist()
        self.local_map.right_cones_x = np.array(local_right)[:,0].tolist()
        self.local_map.right_cones_y = np.array(local_right)[:,1].tolist()

        #update message header
        #self.local_map.header.seq = self.seq
        self.local_map.header.stamp = self.get_clock().now().to_msg()
        self.local_map.header.frame_id = "rslidar"

        with open("out.txt", 'a') as f: 
            print(self.local_map, file=f)

        x_s = np.array(local_left)[:,0].tolist() + np.array(local_right)[:,0].tolist()
        y_s = np.array(local_left)[:,1].tolist() + np.array(local_right)[:,1].tolist()

        plt.scatter(x_s, y_s)
        plt.show()

        self.local_map_pub.publish(self.local_map)

    
    def compareAngle(self, a, b, threshold): # a<b
        mn = min(b-a, 2*np.pi - b + a) # (ex. in degrees): a = 15 and b = 330 are 45 degrees apart (not 315)
        return mn < threshold

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
        curpos = np.array(self.currentstate.carstate[:2])
        heading = self.currentstate.carstate[3]

        ret_localcones_left = []
        ret_localcones_right = []

        # Cones within radius
        close_left = left[((left - curpos) ** 2).sum(axis=1) < radius * radius]
        close_right = right[((right - curpos) ** 2).sum(axis=1) < radius * radius]
        
        # print('close_left cones: ')
        # print(close_left)
        # print('------------------')
        # print('close_right cones:')
        # print(close_right)
        # print('------------------')

        # Concatenate close_left and close_right
        close = np.concatenate((close_left, close_right), axis=0)
        left_len = len(close_left)

        # Calculate delta vectors
        delta_vecs = close - curpos 
        
        # Calculate delta angles
        delta_ang = np.arctan2(delta_vecs[:, 1], delta_vecs[:, 0])
        delta_ang = (delta_ang + 2*np.pi) % (2*np.pi)  # Ensure delta_ang is between 0 and 2*pi

        for i in range(len(delta_ang)):
            if self.compareAngle(min(delta_ang[i], heading), max(delta_ang[i], heading), self.local_vision_delta):
                if i < left_len:
                    ret_localcones_left.append(close[i])
                else:
                    ret_localcones_right.append(close[i])

        return ret_localcones_left, ret_localcones_right


    def solveGraphSlamLock(self):
        self.solving = True 
        x_guess, lm_guess = self.slam.solve_graph()
        self.solving = False
        return x_guess, lm_guess

    def update_recent_cones(self, imu_state, cone_input):
        
        self.perception_backlog_cones += [cone_input]
        self.perception_backlog_imu += [[imu_state[0], imu_state[1]]]

# For running node
def main(args=None):
    rclpy.init(args=args)
    graphslam_global_node = GraphSLAM_Global()
    rclpy.spin(graphslam_global_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()