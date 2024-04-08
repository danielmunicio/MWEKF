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
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

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

from feb_msgs.msg import State, FebPath, Map, Cones
# from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance

class GraphSLAM_Global(Node):
    def __init__(self):

        # ROS2 INTEGRATIONS

        super().__init__('graphslam_global_node')

        # SUBSCRIBERS

        # Handle IMU messages for vehicle state
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            1
        )

        # Handle new cone readings from perception
        self.cones_sub = self.create_subscription(
            Cones,
            '/ground_truth/cones', 
            self.cones_callback,
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

        # SLAM Initialization

        # Initializes a new instance of graphslam from the graphslam
        # Data Association Threshold is to be tweaked
        self.slam = GraphSLAM(solver_type='qp', landmarkTolerance=4)
        
        # used to calculate the state of the vehicle
        self.statetimestamp = 0.0
        self.currentstate = State()
        self.state_seq = 0
        self.cone_seq = 0

        self.global_map = Map()
        self.local_map = Map()

        # radius for which to include local cones ---#UPDATE, perhaps from mpc message
        self.local_radius = 5 
        
        # how far into periphery of robot heading on each side to include local cones (robot has tunnel vision if this is small) (radians)
        self.local_vision_delta = np.pi/2 

        # for handling new messages during the solve step
        self.solving = False

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

        timediff = np.round(timediff, 4)

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
        longitudinal_acc = acc.x
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
        v = Float64()
        v.data = velocity
        heading = Float64()
        heading.data = yaw
        self.currentstate.carstate[0].data += dx[0]
        self.currentstate.carstate[1].data += dx[1]
        self.currentstate.carstate[2] = v
        self.currentstate.carstate[3] = heading
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
        # process time
        dt = self.compute_timediff(imu.header)
        # generate current heading
        roll, pitch, yaw = self.quat_to_euler(imu.orientation)
        # generate current velocity
        velocity = self.compute_velocity(imu.linear_acceleration, dt)
        # for now, we assume velocity is in the direction of heading

        self.currentstate.carstate[2] = velocity

        # generate dx [change in x, change in y] to add new pose to graph
        dx = velocity * dt * np.array([math.cos(yaw), math.sin(yaw)])

        # add new position node to graph
        self.slam.update_position(dx)
        #self.slam.update_backlog_imu(dx)

        # update state msg
        self.update_state(dx, yaw, velocity)

        self.state_pub.publish(self.currentstate)

    def cartesian_to_polar(self, car_state, cone):
        p_x = cone[0] - car_state[0].data
        p_y = cone[1] - car_state[1].data
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
    def cones_callback(self, cones: Cones) -> None: # abt todo: we have had cones as a placeholder message structure yet to be defined (cones.r, cones.theta, cones.color) for now
        # Dummy function for now, need to update graph and solve graph on each timestep
        
        #input cone list & dummy dx since we are already doing that in update_graph with imu data
        cone_matrix = np.hstack(Cones.r, Cones.theta, Cones.color)
        # cone_matrix = [[], [], []]
        # for cone in cones.blue_cones:
        #     r, theta = self.cartesian_to_polar(self.currentstate.carstate[:2], (cone.point.x, cone.point.y))
        #     cone_matrix[0].append(r)
        #     cone_matrix[1].append(theta)
        #     cone_matrix[2].append(2)
        # for cone in cones.yellow_cones:
        #     r, theta = self.cartesian_to_polar(self.currentstate.carstate[:2], (cone.point.x, cone.point.y))
        #     cone_matrix[0].append(r)
        #     cone_matrix[1].append(theta)
        #     cone_matrix[2].append(1)

        # process all new cone messages separately while one thread is solving slam        
        
        #lock
        #self.slam.update_backlog_perception(cone_matrix)
        #if (self.solving):
        #    return

        self.slam.update_graph_color([], cone_matrix, False) # old pre-ros threading
        
        #self.slam.update_graph_color(perception_backlog_imu, perception_backlog_cones)
        #self.perception_backlog_cones = []
        #self.perception_backlog_imu = []
        #self.slam.update_graph_block()
        #x_guess, lm_guess = self.solveGraphSlamLock()
        x_guess, lm_guess = self.slam.solve_graph()

        left_cones = lm_guess[lm_guess[:,2] == 2][:,:2] # blue
        right_cones = lm_guess[lm_guess[:,2] == 1][:,:2] # yellow
        #left_cones = lm_guess[lm_guess[:,2] == 0][:,:2] # orange


        #update map message with new map data 
        self.global_map.left_cones_x = left_cones[:,0] 
        self.global_map.left_cones_y = left_cones[:,1]
        self.global_map.right_cones_x = right_cones[:,0]
        self.global_map.right_cones_y = right_cones[:,1]

        #update message header
        self.cone_seq += 1
        #self.global_map.header.seq = self.seq
        self.global_map.header.stamp = self.get_clock().now().to_msg()
        self.global_map.header.frame_id = "rslidar"

        self.global_map_pub.publish(self.global_map)

        local_left, local_right = self.localCones(self.radius, left_cones, right_cones)

        #update map message with new map data 
        self.local_map.left_cones_x = local_left[:,0] 
        self.local_map.left_cones_y = local_left[:,1]
        self.local_map.right_cones_x = local_right[:,0]
        self.local_map.right_cones_y = local_right[:,1]

        #update message header
        #self.local_map.header.seq = self.seq
        self.local_map.header.stamp = self.get_clock().now().to_msg()
        self.local_map.header.frame_id = "rslidar"

        self.local_map_pub.publish(self.local_map)

    
    def compareAngle(self, a, b, threshold): # a<b
        mn = min(b-a, 2*pi - b + a) # (ex. in degrees): a = 15 and b = 330 are 45 degrees apart (not 315)
        return mn < threshold

    # publishes all cones within given radius
    def localCones(self, radius, left, right):
        left = np.array(left)
        right = np.array(right)
        curpos = np.array(self.current_state.carstate[:2]) # x,y in np
        heading = self.current_state.carstate[3] #calibrated in beginning w initial direction being 0 at (0,0)

        ret_localcones_left = []
        ret_localcones_right = []

        #cones within radius
        close_left = left[((left - curpos) ** 2).sum(axis = 1) < radius * radius]
        close_right = right[((right - curpos) ** 2).sum(axis = 1) < radius * radius]

        close = close_left + close_right
        left_len = len(close_left)

        #cones within 
        delta_vecs = close - curpos 
        
        delta_ang = np.arctan2(delta_vecs[:,1], delta_vecs[:,0]) # 0 to pi when y>0 and 0 to to -pi when y<0
        twopi_arr = np.array([2 * np.pi for i in range(len(delta_ang))])
        twopi_arr[delta_ang>=0]=0
        delta_ang = delta_ang + twopi_arr #0 to 2pi, except 0 is at 3pi/2 in imu perspective - so we adjust so coordinates r synced btwn cones' relative pos & imu

        # delta_ang = delta_ang - (np.pi/2)
        # twopi_arr = np.array([2 * np.pi for i in range(len(at2np))])
        # twopi_arr[delta_ang>=0]=0
        # delta_ang = delta_ang + twopi_arr # 0 to 2pi, same as imu (assuming 0 to 2pi for imu)

        for i in range(len(delta_ang)):
            if(self.compareAngle(min(delta_ang[i],heading), max(delta_ang[i], heading), self.vision_delta)):
                if(i<left_len):
                    ret_localcones_left += [close[i]]
                else:
                    ret_localcones_right += [close[i]]
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