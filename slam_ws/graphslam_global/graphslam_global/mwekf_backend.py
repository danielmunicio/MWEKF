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
from time import perf_counter

from .GraphSLAMSolve import GraphSLAMSolve
from all_settings.all_settings import GraphSLAMSolverSettings as solver_settings
from all_settings.all_settings import GraphSLAMSettings as settings
from all_settings.all_settings import MPCSettings as mpc_settings
from .utility_functions import compute_timediff, quat_to_euler, compute_delta_velocity, cartesian_to_polar
from .icp import run_icp, plot_icp

class MWEKF_Backend():
    def __init__(self, SLAM, x0, num_cones):
        self.SLAM = SLAM
        self.state = x0.reshape(-1, 1)
        # Dynamics Model
        self.n = 4 # num states
        #m = 4 # num measurements - camera, lidar, ws, imu
        self.Q = 1000 * np.eye(self.n)
        self.P = np.eye(self.n)
        self.C = None

        self.R_lidar = np.eye(num_cones)
        self.R_camera = np.eye(num_cones)
        self.R_imu = np.eye(1)
        self.R_wheelspeeds = np.eye(1)

        self.l_f = mpc_settings.L_F
        self.l_r = mpc_settings.L_R
        # Last control input NOTE: last element is time it was recieved at!!!
        sec, nsec = self.SLAM.get_clock().now().seconds_nanoseconds()
        self.last_u = np.array([0., 0., sec + nsec * 1e-9])



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
        pass

    def jac_cones(self, state, cones):
        pass

    def jac_imu(self, state):
        """
        Just taking in Yaw of IMU, so h is just [0 0 0 1 0 0 0 ....]
        """
        jac = np.zeros((1, len(state)))
        jac[0, 3] = 1
        return jac

    def h_imu(self, state):
        return state[3]

    def h_wheelspeeds(self, state):
        return state[2]

    def jac_wheelspeeds(self, state):
        """
        Just taking in velocity, so h is just [0 0 1 0 0 0 ....]
        """
        jac = np.zeros((1, len(state)))
        jac[0, 2] = 1
        return jac

    def approximate_measurement(self, state, measurement, measurement_type):
        if measurement_type == 0 or measurement_type == 1:
            jac = self.jac_cones(state, measurement)
        elif measurement_type == 2:
            jac = self.jac_imu(state)
        elif measurement_type == 3:
            jac = self.jac_wheelspeeds(state)

        else:
            jac = None
        return jac
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
        x, y, v, theta = state
        a, delta = u  # acceleration and steering angle
        
        # The Jacobian terms based on the linearized model
        A = np.zeros((4, 4))

        # Update A matrix according to the linearized model
        A[0, 2] = -v * np.sin(theta)  # ∂(dot_x)/∂theta
        A[0, 3] = np.cos(theta)  # ∂(dot_x)/∂v
        A[1, 2] = v * np.cos(theta)  # ∂(dot_y)/∂theta
        A[1, 3] = np.sin(theta)  # ∂(dot_y)/∂v
        A[2, 3] = np.tan(delta) / (self.l_f + self.l_r)  # ∂(dot_θ)/∂v
        A[3, 3] = 0  # ∂(dot_v)/∂v, since dot_v = a and no dependence on state variables

        return A
    

    def update(self, measurement, measurement_type):
        """
        Updates EKF, based on measurement
        Measurement Types: 
        0 = cones_camera - Nx2 Array of [x y]
        1 = cones_lidar - Nx2 Array of [x y]
        2 = IMU - idk yet
        3 = Wheelspeeds - velocity measurement ? 
        """
        print("U: ", self.last_u)
        print("MEASUREMENT: ", measurement)
        sec, nsec = self.SLAM.get_clock().now().seconds_nanoseconds()
        time = sec + nsec * 1e-9
        dt = time - self.last_u[2]
        x_next = self.g(self.state, self.last_u[0:2], dt)
        print("FIRST POSE GUESS: ", x_next)
        print("DT: ", dt)
        A = self.approximate_A(self.state, u=self.last_u[0:2])
        print("A: ", A)
        P = A @ self.P @ A.T + self.Q
        print("P: ", P)
        # Take jacobian of whichever measurement we're using
        C = self.approximate_measurement(self.state, measurement, measurement_type)
        # Get R based on whichever measurement we're using
        R = self.choose_R(measurement_type)
        print("C SHPAPE: ", C.shape)
        print("R SHAPE: ", R.shape)
        print("SHAPE: ", C @ P @ C.T)
        print("P SHAPE: ", P.shape)
        print("C.T SHAPE: ", C.T.shape)
        print("SHAPE 2: ", (P @ C.T).shape)
        K = P @ C.T @ np.linalg.inv(C @ P @ C.T + R)
        print("KALMAN GAIN: ", K)
        h = self.choose_h(measurement_type)
        print("diff: ", np.array([measurement] - np.array(h(x_next))))

        x_new = x_next + K @ (np.array([measurement]) - np.array([h(x_next)]).reshape(-1, 1))
        
        self.P = (np.eye(self.n) - K @ C) @ P
        self.state = x_new
        print("POSE: ", x_new)

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