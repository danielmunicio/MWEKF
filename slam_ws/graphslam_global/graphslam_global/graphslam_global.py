import numpy as np
from .GraphSLAMFast import GraphSLAMFast
from all_settings.all_settings import GraphSLAMFastSettings as settings
from time import perf_counter
import math
import time
# from numpy.random import random, randn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header, Bool
from sensor_msgs.msg import Imu
from eufs_msgs.msg import WheelSpeedsStamped
from eufs_msgs.msg import CarState
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Pose, Point, Point32
from sensor_msgs.msg import PointCloud


import matplotlib.pyplot as plt 


from feb_msgs.msg import State, FebPath, Map
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance


#TODO
"""
- clean up code
- data association in following laps
    - add every meter, also try by time
- tunable parameters in allsetings/all_settings.py, ros params go here
- lap counter
- drc
    - api for future changes to lap counter/ data association
        - look at ishans for format in auto ee
        - creat function names and what they'll
- add 2d gaussian about landmarks for data assoc
"""

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
            ConeArrayWithCovariance,
            '/fusion/cones', 
            self.cones_callback,
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

        # Once path is finished, turn this node callbacks funcs off
        # self.finish_sub = self.create_subscription(
        #     Bool,
        #     '/path/finished',
        #     self.finish_callback,
        #     1
        # )

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


        ##These are for Visuals in the SIM 
        self.cones_vis_pub = self.create_publisher(
            PointCloud,
            '/slam/conemap',
            1
        )


        self.positionguess = self.create_publisher(
            PointCloud,
            '/slam/guessed_positions',
            1
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/slam/pose',
            1
        )
        # SLAM Initialization

        # Initializes a new instance of graphslam from the graphslam
        # Data Association Threshold is to be tweaked
        self.slam = GraphSLAMFast(**settings)
        
        # used to calculate the state of the vehicle
        self.currentstate_simulator = State()
        self.currentstate_simulator.x = 0.0
        self.currentstate_simulator.y = 0.0
        self.statetimestamp = 0.0
        self.currentstate = State()
        self.currentstate.x = 0.0 
        self.currentstate.y = 0.0
        self.currentstate.velocity = 0.0
        self.currentstate.heading = 0.0
        self.currentstate.lap_count = 0
        self.state_seq = 0.0
        self.cone_seq = 0.0
        self.distance_traveled_danny = 0.0
        self.global_map = Map()
        self.local_map = Map()
        self.lap_counter = 1
        self.LPKRDSM = 4 # LaP oK RaDiuS (Meters)
        self.is_clear_of_lap_count_radius = False
        self.time = time.time()
        self.
        # radius for which to include local cones ---#UPDATE, perhaps from mpc message
        self.local_radius = settings.local_radius
        
        # how far into periphery of robot heading on each side to include local cones (robot has tunnel vision if this is small) (radians)
        self.local_vision_delta = np.pi/2 

        # for handling new messages during the solve step
        self.solving = False

        self.finished = False
        self.usefastslam = False
        self.last_slam_update = np.array([np.Inf, np.Inf])

    def state_sub(self, msg: CarState):
        """ This is a callback function for the SIMULATORS ground truth carstate. 
            Currently being used to get the cars position so we can calculate the cones R, theta properly.
        """
        #with open("sim_data.txt", "a") as f:
            # print("---------------------------------------", file =f)
            # print("GROUND TRUTH CAR STATE: ", file =f)
            # print("x position: ", msg.pose.pose.position.x, file=f)
            # print("y position: ", msg.pose.pose.position.y, file=f)
            # print("heading: ", self.quat_to_euler(msg.pose.pose.orientation)[-1], file=f)
            # print("----------------------------------------", file=f)
        # self.currentstate.x = msg.pose.pose.position.x
        # self.currentstate.y = msg.pose.pose.position.y
        self.currentstate_simulator.x = msg.pose.pose.position.x
        self.currentstate_simulator.y = msg.pose.pose.position.y
        # self.currentstate.velocity = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        return

    def wheelspeed_sub(self, msg: WheelSpeedsStamped):
        self.currentstate.velocity = ((msg.speeds.lb_speed + msg.speeds.rb_speed)/2)*np.pi*0.505/60
        

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
        print("Seconds: ", header.stamp.sec)
        print("NanoSeconds: ", header.stamp.nanosec)
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
    def quat_to_euler(self, quat):#: Quaternion) -> tuple[float, float, float]:
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
    def compute_delta_velocity(self, acc: Vector3, dt: float) -> float:
        # longitudinal_acc = np.linalg.norm([acc.x, acc.y])
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
        # All carstates should be float #'s 
        if self.currentstate.x**2 + self.currentstate.y**2 < self.LPKRDSM**2:
            if self.is_clear_of_lap_count_radius:
                if self.currentstate.lap_count == 0:
                    self.global_map_pub.publish(self.local_map)

                self.currentstate.lap_count += 1
                self.is_clear_of_lap_count_radius = False
        else:
            self.is_clear_of_lap_count_radius = True


        self.currentstate.x += dx[0]
        self.currentstate.y += dx[1]
        self.currentstate.velocity = velocity
        self.currentstate.heading = yaw
        self.state_seq += 1
        #self.currentstate.header.seq = self.state_seq
        self.currentstate.header.stamp = self.get_clock().now().to_msg()
        self.currentstate.header.frame_id = "map"


    """
    Function that takes in IMU messages and processes GraphSLAM based upon 
    Input: imu (Imu_msg)
    - geometry_msgs/Quarternion orientation
    - float64[9] orientation_covariance
    - geometry_msgs/Vector3 angular_velocity
    - float64[9] angular_velocity_covariance
    - geometry_msgs/Vector3 linear_acceleration
    - float64[9] linear_acceleration_covariance

    Note: Depending on whether or not we are using wheel speeds for velocity,
    we should delete the integration of the linear acceleration. 
    """

    def imu_callback(self, imu: Imu) -> None:
        if self.finished:
            return
        # process time
        dt = self.compute_timediff(imu.header)
        if (dt > 1):
            return

        # generate current heading
        roll, pitch, yaw = self.quat_to_euler(imu.orientation)

        # generate current velocity
        delta_velocity = self.compute_delta_velocity(imu.linear_acceleration, dt)
        velocity = self.currentstate.velocity + delta_velocity
        self.currentstate.velocity = velocity

        # for now, we assume velocity is in the direction of heading
        # generate dx [change in x, change in y] to add new pose to graph
        dx = self.currentstate.velocity * dt * np.array([math.cos(yaw), math.sin(yaw)])
        
        # add new position node to graph
        # self.slam.update_position(dx)
        #self.slam.update_backlog_imu(dx)

        # update state msg
        # self.currentstate.heading = yaw
        # self.currentstate.heading = yaw
        self.update_state(dx, yaw, self.currentstate.velocity)
        
        ## Show the estimated Pose on the Sim        
        pose_msg = PoseStamped()
        # pose_msg.pose = Pose()
        # pose_msg.pose.position = Point()
        # pose_msg.pose.orientation = Quaternion()
 
        pose_msg.pose.position.x = self.currentstate.x
        pose_msg.pose.position.y = self.currentstate.y
        pose_msg.pose.position.z = 0*self.currentstate.velocity
        pose_msg.pose.orientation.w = np.cos(self.currentstate.heading/2)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = np.sin(self.currentstate.heading/2)
 
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        print("Velocity!", self.currentstate.velocity)
        self.pose_pub.publish(pose_msg)

        ## Show Estimated Pose END 

        self.state_pub.publish(self.currentstate)

        delta_pos = dt * self.currentstate.velocity
        self.distance_traveled_danny += delta_pos

        # print(f"TIME TAKEN FOR IMU CALLBACK: {times}")

    def cartesian_to_polar(self, car_state, cone):
        p_x = cone[0] - car_state[0]
        p_y = cone[1] - car_state[1]
        r = math.sqrt(p_x**2 + p_y**2)
        angle = math.atan2(p_y, p_x)
        return r, angle
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
        bloobs = np.array([[i.point.x for i in cones.blue_cones],
                           [i.point.y for i in cones.blue_cones]])
        yellow = np.array([[i.point.x for i in cones.yellow_cones],
                           [i.point.y for i in cones.yellow_cones]])
        

        carstate_array = [self.currentstate.x, self.currentstate.y, self.currentstate.velocity, self.currentstate.heading]
        rot = lambda theta: np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        pos = np.array(carstate_array[:2])[:, np.newaxis]
        bloobs = rot(-self.currentstate.heading)@bloobs + pos
        yellow = rot(-self.currentstate.heading)@yellow + pos

        self.local_map.left_cones_x = list(bloobs[0])
        self.local_map.left_cones_y = list(bloobs[1])
        self.local_map.right_cones_x = list(yellow[0])
        self.local_map.right_cones_y = list(yellow[1])
        # self.local_map_pub.publish(self.local_map)

        # with open("sim_data.txt", "a") as f:
        #     print("From SLAM cones map:", self.local_map, file=f)
        #     print(file=f)

    
        # x_s = [cone.point.x for cone in cones.blue_cones] + [cone.point.x for cone in cones.yellow_cones]
        # y_s = [cone.point.y for cone in cones.blue_cones] + [cone.point.y for cone in cones.yellow_cones]

        # plt.scatter(x_s, y_s)
        # plt.show()

        # Dummy function for now, need to update graph and solve graph on each timestep
        if self.finished:
            return
        
        #input cone list & dummy dx since we are already doing that in update_graph with imu data
        # cone_matrix = np.hstack(Cones.r, Cones.theta, Cones.color)
        cone_matrix = [[], [], []]
        for cone in cones.blue_cones:
            r, theta = self.cartesian_to_polar([0.0, 0.0], (cone.point.x, cone.point.y))
            cone_matrix[0].append(r)
            cone_matrix[1].append(theta)
            cone_matrix[2].append(2)
        for cone in cones.yellow_cones:
            r, theta = self.cartesian_to_polar([0.0, 0.0], (cone.point.x, cone.point.y))
            cone_matrix[0].append(r)
            cone_matrix[1].append(theta)
            cone_matrix[2].append(1)
        # for cone in cones.big_orange_cones:
        #     r, theta = self.cartesian_to_polar([0.0, 0.0], (cone.point.x, cone.point.y))
        #     cone_matrix[0].append(r)
        #     cone_matrix[1].append(theta)
        #     cone_matrix[2].append(0)

        cone_matrix = np.array(cone_matrix).T
        # print("CONESHAPE", cone_matrix.shape)
        cone_dx = cone_matrix[:,0] * np.cos(cone_matrix[:,1]+self.currentstate.heading) # r * cos(theta) element wise
        cone_dy = cone_matrix[:,0] * np.sin(cone_matrix[:,1]+self.currentstate.heading) # r * sin(theta) element_wise
        cartesian_cones = np.vstack((cone_dx, cone_dy, cone_matrix[:,2])).T # n x 3 array of n cones and dx, dy, color   -- input for update_graph


        # cone_matrix = np.hstack([np.vstack([bloobs.T, yellow.T]), np.array([[0]*len(bloobs[0]) + [1]*len(yellow[0])]).T])
        # process all new cone messages separately while one thread is solving slam        
        
        #lock
        #self.slam.update_backlog_perception(cone_matrix)
        #if (self.solving):
        #    return
        #if np.linalg.norm(self.last_slam_update-np.array([self.currentstate.x, self.currentstate.y])) > 2:
            #self.slam.update_graph(np.array([self.currentstate.x, self.currentstate.y])-self.last_slam_update if self.last_slam_update[0]<999999999.0 else np.array([0.0, 0.0]), cartesian_cones[:, :2], cartesian_cones[:, 2].flatten()) # old pre-ros threading
            #print(cartesian_cones.T.shape)
            #self.last_slam_update = np.array([self.currentstate.x, self.currentstate.y])

        if (self.time - time.time() > 10000): #NOTE self.time - time.time() should be negative

            # last_slam_update initialized to infinity, so set current state x,y to 0 in the case. otherwise, update graph with relative position from last update graph
            self.slam.update_graph(np.array([self.currentstate.x, self.currentstate.y])-self.last_slam_update if self.last_slam_update[0]<999999999.0 else np.array([0.0, 0.0]), 
                                   cartesian_cones[:, :2], 
                                   cartesian_cones[:, 2].flatten()) # old pre-ros threading
            print(cartesian_cones.T.shape)
            self.last_slam_update = np.array([self.currentstate.x, self.currentstate.y])
            self.time = time.time()
            print("UPDATING STATE")
        else:
            return
        #self.slam.update_graph_color(perception_backlog_imu, perception_backlog_cones)
        #self.perception_backlog_cones = []
        #self.perception_backlog_imu = []
        #self.slam.update_graph_block()
        #x_guess, lm_guess = self.solveGraphSlamLock()
        
        # x_guess, lm_guess = self.slam.solve_graph()
        
        x_guess, lm_guess = self.slam.xhat, np.hstack((self.slam.lhat, self.slam.color[:, np.newaxis]))

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
        
        pos = np.array(x_guess[-1]).flatten()
        self.currentstate.x = pos[0]
        self.currentstate.y = pos[1]

        #x and lm guess come out as lists, so change to numpy arrays
        x_guess = np.array(x_guess)
        lm_guess = np.array(lm_guess)
        

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

        local_left, local_right = self.localCones(self.local_radius*0 + 20, left_cones, right_cones)
        print("here8")
        if (len(np.array(local_left)) == 0 or len(np.array(local_right)) == 0):
            print("here9")
            return
        print("here10")
        # local_left, local_right = left_cones, right_cones

        # print('len of local left and local right cones:')
        # print(len(local_left))
        # print(len(local_right))
        # print('local left: ')
        # print(local_left)
        # print('local right: ')
        # print(local_right)
        #update map message with new map data 

        self.local_map.left_cones_x = np.array(local_left)[:,0].tolist()
        self.local_map.left_cones_y = np.array(local_left)[:,1].tolist()
        self.local_map.right_cones_x = np.array(local_right)[:,0].tolist()
        self.local_map.right_cones_y = np.array(local_right)[:,1].tolist()

        #update message header
        #self.local_map.header.seq = self.seq
        self.local_map.header.stamp = self.get_clock().now().to_msg()
        self.local_map.header.frame_id = "map"

        print("here11")
        self.local_map_pub.publish(self.local_map)
        print("here12")
    
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
        curpos = np.array([self.currentstate.x, self.currentstate.y])
        heading = self.currentstate.heading

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
        delta_vecs = (np.array([[np.cos(-heading), -np.sin(-heading)],
                                 [np.sin(-heading), np.cos(-heading)]])@(delta_vecs.T)).T
        # Calculate delta angles
        # delta_ang = np.arctan2(delta_vecs[:, 1], delta_vecs[:, 0])
        # delta_ang = (delta_ang + 2*np.pi) % (2*np.pi)  # Ensure delta_ang is between 0 and 2*pi
        mask = delta_vecs[:, 0]>0

        to_plot = close[mask]
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
        # self.cones_vis_pub.publish(cones_msg)
        print("local cones done!!")



        return close[:left_len][mask[:left_len]], close[left_len:][mask[left_len:]]
        for i in range(len(delta_ang)):
            if self.compareAngle(min(delta_ang[i], heading), max(delta_ang[i], heading), self.local_vision_delta):
                if i < left_len:
                    ret_localcones_left.append(close[i])
                else:
                    ret_localcones_right.append(close[i])

        return ret_localcones_left, ret_localcones_right


# For running node
def main(args=None):
    rclpy.init(args=args)
    graphslam_global_node = GraphSLAM_Global()
    rclpy.spin(graphslam_global_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
