import numpy as np
from numpy.linalg import *
# import matplotlib.pyplot as plt
# import csv
# import track_generator_leonid.track_main as trackmain
# import track_generator_leonid.settings as settings
import graphslam_colorsolve as slamLib
from time import perf_counter
import math
# from numpy.random import random, randn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

from feb_msgs.msg import State
from feb_msgs.msg import Map
from feb_msgs.msg import FebPath

class GraphSLAM_Global(Node):
    def __init__(self):

        # ROS2 INTEGRATIONS

        super().__init__('graphslam_global_node')

        # SUBSCRIBERS

        # Handle IMU messages for vehicle state
        self.imu_sub = self.create_subscription(
            Imu,
            '/odometry/Imu',
            self.imu_callback,
            1
        )

        # Handle new cone readings from perception
        self.cones_sub = self.create_subscription(
            List_of_Cones,
            '/perception/YOLO/Cones', # To be changed
            self.cones_callback,
            1
        )

        # PUBLISHERS
        
        # Publish the current vehicle's state: X, Y, Velo, Theta
        self.state_pub = self.create_publisher(
            State,
            '/slam/State',
            1
        )

        # Publish the current map (GLOBAL_NODE, so this will send the whole map)
        self.map_pub = self.create_publisher(
            Map, 
            '/slam/Global-Map',
            1
        )

        # SLAM Initialization

        # Initializes a new instance of graphslam from the graphslam
        # Data Association Threshold is to be tweaked
        self.slam = slamLib.GraphSLAM(solver_type='qp', landmarkTolerance=4)
        
        # used to calculate the state of the vehicle
        self.statetimestamp = 0.0
        self.currentstate = State()
        self.map = Map()
        self.seq = 0
    
    

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
        self.currentstate.carstate[0] += dx[0]
        self.currentstate.carstate[1] += dx[1]
        self.currentstate.carstate[2] = velocity
        self.currentstate.carstate[3] = yaw


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
        pass
        # process time
        dt = self.compute_timediff(imu.header)
        # generate current heading
        roll, pitch, yaw = self.quat_to_euler(imu.orientation)
        self.currentstate.heading = yaw
        # generate current velocity
        velocity = self.compute_velocity(imu.linear_acceleration, dt)
        # for now, we assume velocity is in the direction of heading

        self.currentstate.velocity = velocity

        # generate dx [change in x, change in y] to add new pose to graph
        dx = velocity * dt * np.array([math.cos(yaw), math.sin(yaw)])

        # add new position node to graph
        self.slam.update_position(dx)

        # update state msg
        self.update_state(dx, yaw, velocity)

        self.state_pub.publish(self.currentstate)



    """
    Function that takes the list of cones, updates and solves the graph
    Input:
        List_of_Cones: (n x 3 list of n cones, defined by their r (distance from car), theta (angle from car heading), and color
    """
    def cones_callback(self, cones: List_of_Cones) -> None:
        # Dummy function for now, need to update graph and solve graph on each timestep
        pass

        #input cone list & dummy dx since we are already doing that in update_graph with imu data
        self.slam.update_graph_color([],List_of_Cones, False)
        x_guess, lm_guess = self.slam.solve_graph()

        left_cones = lm_guess[lm_guess[:,2] == 2][:,:2] # blue
        right_cones = lm_guess[lm_guess[:,2] == 1][:,:2] # yellow
        #left_cones = lm_guess[lm_guess[:,2] == 0][:,:2] # orange


        #update map message with new map data 
        self.map.left_cones_x = left_cones[:,0] 
        self.map.left_cones_y = left_cones[:,1]
        self.map.right_cones_x = right_cones[:,0]
        self.map.right_cones_y = right_cones[:,1]

        #update message header
        self.seq += 1
        self.map.header.seq = self.seq
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.header.frame_id = "cone_locations"

        self.map_pub.publish(self.map)

# For running node
def main(args=None):
    rclpy.init(args=args)
    graphslam_global_node = GraphSLAM_Global(Node)
    rclpy.spin(graphslam_global_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()