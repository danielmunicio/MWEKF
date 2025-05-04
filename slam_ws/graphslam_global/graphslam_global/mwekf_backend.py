# Ros Imports 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Header, Bool
from eufs_msgs.msg import WheelSpeedsStamped, CarState, ConeArrayWithCovariance, ConeWithCovariance
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Pose, Point, Point32
from sensor_msgs.msg import PointCloud, Imu
from feb_msgs.msg import State, FebPath, Map, Cones, ConesCartesian

# Python Libraries
import numpy as np


from .GraphSLAMSolve import GraphSLAMSolve
from all_settings.all_settings import GraphSLAMSolverSettings as solver_settings
from all_settings.all_settings import GraphSLAMSettings as settings
from all_settings.all_settings import MPCSettings as mpc_settings
from .utility_functions import compute_timediff, quat_to_euler, compute_delta_velocity, cartesian_to_polar
from .icp import run_icp, plot_icp

class MWEKF_Backend():
    def __init__(self, num_cones):
        # Dynamics Model
        n = 4 + num_cones # num states
        #m = 4 # num measurements - camera, lidar, ws, imu
        self.Q = np.eye(n)
        self.P = np.eye(n)
        self.C = None

        self.last_time = None



    #################
    # Different Measurement Models 
    #################

    def cones_measurement_linearization(self, x, cones):
        """
        Calculates partial derivative of measurement for cones. 
        Remind me to do partial derivative tmmrw
        """
        pass

    def h_cones(self, state, cones):
        """
        Calculates the expected position of the cones, based on the position of the car
        Args: 
        state - state vec
        cones - cones measurement, NOTE: values of 0 or -1 or something will be cones not spotted
        Cones not spotted should not be included in H
        # Still need to figure out the masking logic or something
        Returns: 2n x 1 array of cones
        """
    def jac_cones(self, state, cones):
        pass

    def jac_imu(self, state):
        """
        Just taking in Yaw of IMU, so h is just [0 0 0 1 0 0 0 ....]
        """
        jac = np.zeros((1, len(state)))
        jac[0, 3] = 1
        return jac

    def jac_wheelspeeds(self, state):
        """
        Just taking in velocity, so h is just [0 0 1 0 0 0 ....]
        """
        jac = np.zeroes((1, len(state)))
        jac[0, 2] = 1
        return jac

    def approximate_imu_measurement(self, msg):
        """
        Takes in IMU measurement, and just 
        """
        pass

    def approximate_wheelspeed_measurement(self):
        pass

    def approximate_measurement(self, state, measurement, measurement_type):
        if measurement_type == 0 or 1:
            jac = self.jac_cones(state, measurement, measurement_type)
        elif measurement_type == 2:
            jac = self.jac_imu(state)
        elif measurement_type == 3:
            jac = self.jac_wheelspeeds(state)
        else:
            jac = None

    ###############
    # State Estimation based on Dynamics
    # Might add multiple state estimation steps based on dynamics before doing update, to fully utilize MPC speed
    ################
    def g(self, state, u, dt, l_r=mpc_settings.L_R, l_f=mpc_settings.L_F):
        """
        Get next state using MPC dynamics model
        state: [x, y, v, phi, ...]
        u: [a, psi] - acceleration and steering angle
        """
        x, y, v, phi = state[0], state[1], state[2], state[3]
        a, psi = u[0], u[1]

        beta = np.arctan((l_r * np.tan(psi)) / (l_f + l_r))

        dx = v * np.cos(phi + beta)
        dy = v * np.sin(phi + beta)
        dv = a
        dphi = (v / l_r) * np.sin(beta)
        
        # Copy last state, to keep all cones the same
        state_next = state.copy()
        state_next[0] = x + dx * dt
        state_next[1]= y + dy * dt
        state_next[2] = v + dv * dt
        state_next[3] = phi + dphi * dt

        return state_next

    def approximate_A(self, state, u):
        pass


    def update(self, state, u, measurement, measurement_type, dt):
        """
        Updates EKF, based on measurement
        Measurement Types: 
        0 = cones_camera - Nx2 Array of [x y]
        1 = cones_lidar - Nx2 Array of [x y]
        2 = IMU - idk yet
        3 = Wheelspeeds - velocity measurement ? 
        """
        x_next = self.g(state, u, dt)
        A = self.approximate_A(state, u)
        P = A @ self.P @ A.T + self.Q

        # Take jacobian of whichever measurement we're using
        C = self.approximate_measurement(state, measurement, measurement_type)
        # Get R based on whichever measurement we're using
        R = self.choose_R(measurement_type)

        K = P @ C.T @ np.linalg.inv(C @ P @ C.T + R)
        h = self.choose_h(measurement_type)
        x_new = x_next + K @ (measurement - h(x_next))
        self.P = (np.eye(self.n) - K @ C) @ P
        
        return x_new

    def choose_R(self, measurement_type):
        if measurement_type == 0:
            return self.R_camera
        if measurement_type == 1:
            return self.R_lidar
        if measurement_type == 2:
            return self.R_imu
        if measurement_type == 3:
            return self.R_wheelspeeds
        
    def choose_h(self, measurement_type):
        if measurement_type == 0:
            return self.h_cones
        if measurement_type == 1:
            return self.h_cones
        if measurement_type == 2:
            return self.h_imu
        if measurement_type == 3:
            return self.h_wheelspeeds