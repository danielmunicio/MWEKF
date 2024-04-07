import numpy as np
from numpy.linalg import *

from time import perf_counter
import math
# from numpy.random import random, randn

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

from feb_msgs.msg import State, FebPath, Map, cones

from fastslam_utils import *

class FastSLAM(Node):

    def __init__(self):
        super().__init__('fastslam_node')

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
            cones,
            '/perception/cones', # To be changed
            self.cones_callback,
            1
        )

        # This is only used to store the last map. Once path is finished the last map is processed 
        # and this becomes a useless subscription
        self.map_sub = self.create_subscription(
            Map,
            '/slam/map/global',
            self.map_callback,
            1
        )

        # This is only used to store the latest state. Once path is finished, this will be useless
        #NOTE: Make sure we don't run into issues where we are processing our own topics
        self.state_sub = self.create_subscription(
            State,
            '/slam/state',
            self.state_callback,
            1
        )

        self.finish_sub = self.create_subscription(
            bool,
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

        # Map is completed at this point, only publish state within map

        # SLAM Initializations

        # self.particles = [Particle(0, 0, 1 / N, 0) for _ in range(N)]
        # used to calculate the state of the vehicle
        self.statetimestamp = 0.0
        self.currentstate = State()
        self.state_seq = 0
        self.map_finished = False
        self.last_map = None
        self.fastslam = None

    # Callback for IMU
    # Needs all the other functions used in the graphslam imu processing
    # copy pasted from there
        
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
    def update_state(self, loc: tuple, yaw: float, velocity: float) -> None:
        self.currentstate.carstate[0] = loc[0]
        self.currentstate.carstate[1] = loc[1]
        self.currentstate.carstate[2] = velocity
        self.currentstate.carstate[3] = yaw
        self.state_seq_seq += 1
        self.currentstate.header.seq = self.seq
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
        if self.map_finished and self.fastslam is not None:
            # process time
            dt = self.compute_timediff(imu.header.stamp)
            # generate current heading
            roll, pitch, yaw = self.quat_to_euler(imu.orientation)
            # generate current velocity
            velocity = self.compute_velocity(imu.linear_acceleration, dt)
            # for now, we assume velocity is in the direction of heading

            # generate dx [change in x, change in y] to add new pose to graph
            dx = velocity * dt * np.array([math.cos(yaw), math.sin(yaw)])
            dtheta = yaw - self.currentstate.carstate[3]


            # THIS IS WHERE FASTSLAM WILL BE CALLED USING THE PARTICLES
            dposition = (dx[0], dx[1], dtheta)
            particles = self.fastslam.predict_poses(dposition)
            best_particle = self.fastslam.get_heightest_wegit(particles)

            # update state msg
            self.update_state((best_particle.x, best_particle.y), best_particle.theta, velocity)

            self.state_pub.publish(self.currentstate)

    # Cone Callback will only be used to update weights
    # Only IMU Callback will return a new state
    # Without the Cone data, there will be no changing of 
    # weights because there is nothing to compare particles to
    def cones_callback(self, cones: cones) -> None:
        if self.map_finished and self.fastslam is not None:
            z = cones.cones
            # We update the weights of the particles based on the cone data
            # remains to be seen how we need to process these depending on the perception data
            self.fastslam.update_weights(z)

    # Callback for Map
    def map_callback(self, map: Map) -> None:
        if not self.map_finished:
            self.last_map = map
        
    # Callback for State
    def state_callback(self, state: State) -> None:
        if not self.map_finished:
            self.currentstate = state

    # Signal that map is completed
    # This should initialize fastslam because graphslam is completed
    def finish_callback(self, finished: bool) -> None:
        if not self.map_finished:
            self.map_finished = finished
            # Above 2 lines check if the map is just being completed
            # If so, we initialize SLAM
            if self.map_finished:
                # Process carstate into tuple (x, y, theta)
                x = self.currentstate.carstate[0]
                y = self.currentstate.carstate[1]
                theta = self.currentstate.carstate[3]
                # Initialize FastSLAM
                self.fastslam = FastSLAM(self.last_map, (x, y, theta))

# For running node
def main(args=None):
    rclpy.init(args=args)
    fastslam_node = FastSLAM(Node)
    rclpy.spin(fastslam_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()