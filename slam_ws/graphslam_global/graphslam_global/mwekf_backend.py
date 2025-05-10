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
    def __init__(self, SLAM, x0):
        self.SLAM = SLAM
        self.state = x0[:, np.newaxis]
        self.start_time = perf_counter()
        self.last_called_time = perf_counter()
        # Dynamics Model
        self.num_cones = 0
        self.n = 4 # num state

        self.Q = np.eye(self.n)
        self.P = np.eye(self.n)
        self.C = None
        self.P_cones = None
        self.R_lidar = np.eye(1)
        self.R_camera = np.eye(1)
        self.R_imu = np.eye(1)
        self.R_wheelspeeds = np.eye(1)
        self.R_SLAM = np.eye(2)

        self.l_f = mpc_settings.L_F
        self.l_r = mpc_settings.L_R

        # Last control input NOTE: last element is time it was recieved at!!!
        self.last_u = np.array([0., 0., perf_counter()])


        # n x 3 array of [x, y, color]
        self.cones = None
        self.current_cones_message = None
    #################
    # Different Measurement Models 
    #################

    def cones_measurement_linearization(self, x, cones):
        """
        Calculates partial derivative of measurement for cones. 
        Remind me to do partial derivative tmmrw
        """
        pass

    def h_cones(self, state):
        """
        Calculates the expected position of the cones, based on the position of the car
        Args: 
        state - state vec
        Cones not spotted should not be included in H
        Returns: 2n x 1 array of cones
        """
        cones = self.current_cones_message # n x 4 of global index, x, y, color
        cones_to_grab = []
        for idx in self.current_cones_message[:, 0]:
            idx_in_mwekf = np.where(self.cone_indices == idx)[0][0]
            cones_to_grab.append(self.cones[idx_in_mwekf, 0:2])
        
        cones_to_grab = np.array(cones_to_grab)
        print("CONES TO GRAB: ", cones_to_grab)
        # translate them into the local frame of the car
        x, y, = state[0:2]
        heading = state[3]

        
        translated = cones_to_grab - np.array([x, y]).flatten()

        # Rotate to local frame using inverse rotation
        c, s = np.cos(-heading), np.sin(-heading)
        rot_matrix = np.array([[c, -s], [s, c]])
        rotated = translated @ rot_matrix.T

        return rotated


    def jac_cones(self, state, cones):
        """
        Computes the Jacobian of cone measurements (in the car frame) w.r.t. the full state.
        
        Inputs:
            - state: (4,) array with [x, y, velocity, heading]
            - cones: (n, ≥3) array with cone info (only x, y used)
        
        Returns:
            - H: (2n, 4 + 2n) Jacobian matrix
        """
        x, y, v, theta = state
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # Grab only x, y positions of cones
        cones_organized = cones[:, 1:3]  # shape (n, 2)
        num_cones = cones_organized.shape[0]
        print("CONES MEASUREMENT: ", cones)
        # Augment the state vector
        cone_flat = cones_organized.flatten()  # shape (2n,)
        print("STATE: ", state.flatten())
        print("CONES FLAT: ", cone_flat)
        full_state = np.concatenate([state.flatten(), cone_flat])  # shape (4 + 2n,)
        print("FULL STATE: ", full_state)
        print("SELF.CONES: ", self.cones)
        H = np.zeros((2 * num_cones, 4 + 2 * num_cones))  # Jacobian
        print("H SHAPE: ", H.shape)
        for i in range(num_cones):
            cone_x = cones_organized[i, 0]
            cone_y = cones_organized[i, 1]

            dx = cone_x - x
            dy = cone_y - y

            row = 2 * i
            col_x = 4 + 2 * i
            col_y = 4 + 2 * i + 1

            # ∂h/∂x
            H[row, 0] = -cos_theta
            H[row + 1, 0] = sin_theta

            # ∂h/∂y
            H[row, 1] = -sin_theta
            H[row + 1, 1] = -cos_theta

            # ∂h/∂theta
            H[row, 3] = -sin_theta * dx + cos_theta * dy
            H[row + 1, 3] = -cos_theta * dx - sin_theta * dy

            # ∂h/∂cone_x
            H[row, col_x] = cos_theta
            H[row + 1, col_x] = -sin_theta

            # ∂h/∂cone_y
            H[row, col_y] = sin_theta
            H[row + 1, col_y] = cos_theta
        return H


    def jac_imu(self, state):
        """
        Just taking in Yaw of IMU, so h is just [0 0 0 1 0 0 0 ....]
        """
        jac = np.zeros((1, len(state)))
        jac[0, 3] = 1
        return jac

    def h_imu(self, state):
        return np.array([state[3]])

    def h_wheelspeeds(self, state):
        return np.array([state[2]])

    def h_SLAM(self, state):
        return state[0:2]

    def jac_wheelspeeds(self, state):
        """
        Just taking in velocity, so h is just [0 0 1 0 0 0 ....]
        """
        jac = np.zeros((1, len(state)))
        jac[0, 2] = 1
        return jac

    def jac_SLAM(self, state):
        jac = np.zeros((2, len(state)))
        jac[0, 0] = 1
        jac[1, 1] = 2
        return jac

    def approximate_measurement(self, state, measurement, measurement_type):
        if measurement_type == 0 or measurement_type == 1:
            jac = self.jac_cones(state, measurement)
        elif measurement_type == 2:
            jac = self.jac_imu(state)
        elif measurement_type == 3:
            jac = self.jac_wheelspeeds(state)
        elif measurement_type == 4:
            jac = self.jac_SLAM(state)
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
        state = state.flatten()
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
        
        return state_next[:, np.newaxis]


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
    
    def compute_A_fd(self, x, u, dt, eps=1e-5):
        n = len(x)
        A = np.zeros((n, n))
        fx = self.g(x, u, dt)
        for i in range(n):
            dx = np.zeros((n, 1))
            dx[i] = eps
            fx_plus = self.g((x + dx), u, dt)
            fx_minus = self.g((x - dx), u, dt)
            A[:, i] = ((fx_plus - fx_minus) / (2 * eps)).flatten()

        return A

    def update(self, measurement, measurement_type):
        """
        Updates EKF, based on measurement
        Measurement Types: 
        0 = cones_camera - Nx2 Array of [x y]
        1 = cones_lidar - Nx2 Array of [x y]
        2 = IMU - idk yet
        3 = Wheelspeeds - velocity measurement ? 
        4 = SLAM state array of [x y]
        5 = SLAM cones
        """
        if (measurement_type == 0) or (measurement_type == 1):
            self.update_cones(measurement, measurement_type)
            return
        if measurement_type == 4:
            return
        time = perf_counter()
        dt = time - self.last_u[2]
        self.last_u[2] = perf_counter()
        x_next = self.g(self.state, self.last_u[0:2], dt)
        A = self.approximate_A(self.state, u=self.last_u[0:2])
        #A = self.compute_A_fd(self.state, self.last_u[0:2], self.last_u[2])
        P = A @ self.P @ A.T + self.Q
        # Take jacobian of whichever measurement we're using
        C = self.approximate_measurement(x_next, measurement, measurement_type)
        # Get R based on whichever measurement we're using
        R = self.choose_R(measurement_type)
        K = P @ C.T @ np.linalg.inv(C @ P @ C.T + R)
        h = self.choose_h(measurement_type)
        x_new = x_next + K @ (measurement.reshape(-1, 1) - h(x_next).reshape(-1, 1))
        x_diff = x_new[0] - self.state[0]
        self.P = (np.eye(self.n) - K @ C) @ P
        self.state = x_new

    def update_cones(self, measurement, measurement_type):
        return
        print("CONES MEASUREMENT: ", measurement.shape)
        print("CURRENT CONES: ", self.cones.shape)
        self.num_cones_in_measurement = len(measurement[:, 0])
        self.current_cones_message = measurement
        time = perf_counter()
        dt = time - self.last_u[2]
        self.last_u[2] = perf_counter()
        x_next = self.g(self.state, self.last_u[0:2], dt)
        A = self.approximate_A(self.state, u=self.last_u[0:2])
        
        # Create the zero matrix for cones' movement
        A_cones = np.zeros((2 * self.num_cones_in_measurement, 2 * self.num_cones_in_measurement))
        
        # Extend A to account for cone states (zeros for cone states, A for vehicle states)
        A_part_1 = np.hstack([A, np.zeros((4, 2 * self.num_cones_in_measurement))])
        A_part_2 = np.hstack([np.zeros((2 * self.num_cones_in_measurement, 4)), A_cones])
        
        # Combine both parts into the final A matrix
        A_extended = np.vstack([A_part_1, A_part_2])

        print("NUM CONES: ", self.num_cones_in_measurement)
        print("A SHAPE: ", A_extended.shape)
        # NOTE: A is 4x4, because the original state is only x y v heading
        # Add cones approximation (Cones in global frame so they wont move at all)
        Q = self.get_Q_cones()
        print("Q SHAPE: ", Q.shape)
        print("P SHAPE: ", self.P_cones.shape)
        P = A_extended @ self.P_cones @ A_extended.T + Q
        # Take jacobian of whichever measurement we're using
        C = self.approximate_measurement(x_next, measurement, measurement_type)
        # Get R based on whichever measurement we're using
        R = self.choose_R(measurement_type)
        print("P: ", P.shape)
        print("C: ", C.shape)
        K = P @ C.T @ np.linalg.inv(C @ P @ C.T + R)
        h = self.choose_h(measurement_type)
        print("MEASUREMENT SHAPE: ", measurement[:, 0:2].reshape(-1, 1).shape)
        print("OTHER SHAPE: ", h(x_next).reshape(-1, 1).shape)
        print("K SHAPE: ", K.shape)
        x_adding =  K @ (measurement[:, 0:2].reshape(-1, 1) - h(x_next).reshape(-1, 1))
        print("X ADDING: ", x_adding.shape)
        x_new = x_adding[0:4, :] + x_next
        # Add the x and y to cones, have to do some funny stuff
        # self.cones[:, 1:3] gives all the cones in x y pairs, while x_adding is x y x y x y as one long array
        print("SHAPE: ", x_adding[4:].shape)
        cone_corrections = x_adding[4:].reshape(-1, 2)
        print("SELF.CONES SHAPE: ", self.cones.shape)
        cones_new = self.cones[:, 1:3] + cone_corrections

        self.P = (np.eye(self.n) - K @ C) @ P
        self.state = x_new
        self.cones[:, 1:3] = cones_new


    def get_Q_cones(self):
        self.P_cones = np.eye(4 + 2 * self.num_cones_in_measurement)
        return np.eye(4 + 2 * self.num_cones_in_measurement)

    def choose_R(self, measurement_type):
        if measurement_type == 0:
            return np.eye(2 * self.num_cones_in_measurement) 
        if measurement_type == 1:
            return np.eye(2 * self.num_cones_in_measurement)
        if measurement_type == 2:
            return self.R_imu
        if measurement_type == 3:
            return self.R_wheelspeeds
        if measurement_type == 4:
            return self.R_SLAM

    def choose_h(self, measurement_type):
        if measurement_type == 0:
            return self.h_cones
        if measurement_type == 1:
            return self.h_cones
        if measurement_type == 2:
            return self.h_imu
        if measurement_type == 3:
            return self.h_wheelspeeds
        if measurement_type == 4:
            return self.h_SLAM

    def add_cones(self, cones):
        """
        Add cones to MWEKF window
        Cones: list of elements (x, y, color)
        """
        print("ADDING CONES")
        if self.cones is None:
            self.cones = cones
            return
        self.cones = np.vstack([self.cones, cones])


    def get_cones(self):
        """
        Returns the cones to add to SLAM Graph
        n x 3 array of n cones, x, y, color
        """
        return self.cones
