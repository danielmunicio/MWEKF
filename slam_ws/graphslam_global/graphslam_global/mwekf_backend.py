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

        self.R_lidar = np.eye(1)
        self.R_camera = np.eye(1)
        self.R_imu = np.eye(1)
        self.R_wheelspeeds = np.eye(1)
        self.R_SLAM = np.eye(2)

        self.l_f = mpc_settings.L_F
        self.l_r = mpc_settings.L_R

        # Last control input NOTE: last element is time it was recieved at!!!
        self.last_u = np.array([0., 0., perf_counter()])

        # What cones we have, stored as indices in the global map
        self.cone_indices = []
        # the actual cone positions, stored as 
        self.cones = None

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
        idxs = cones[:, 0]
        pts = cones[:, 1:3]
        

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
        4 = SLAM update
        """
        num_cones = 0
        if (measurement_type == 0) or (measurement_type == 4) or (measurement_type == 1):
            num_cones = len(measurement[:, 0])
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

    def choose_R(self, measurement_type, measurement_length=0):
        if measurement_type == 0:
            return self.R_camera
        if measurement_type == 1:
            return self.R_lidar
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
        Cones: list of elements (num, x, y, color)
        # num = -1 represents the fact that they are NOT in global map yet
        # num = map_idx means that they ARE in the global map
        """
        print("CONES ADDED TO WINDOW: ", cones)
        if self.cones is None:
            self.cones = cones[:, 1:4]
            self.cone_indices = cones[:, 0]
            return
        self.cones = np.vstack([self.cones, cones[:, 1:4]])
        self.cone_indices = np.hstack([self.cone_indices, cones[:, 0]])

    def remove_cones(self, cone_indices):
        """
        Args: cone_indices: index of cones in global map to remove from the mwekf
        Should remove the cones from the MWEKF, update everything accordingly
        Should go through self.cones_indices, and remove the cone in self.cones_indices and self.cones
        If its index is in cone_indices
        """
        cone_indices = np.array(cone_indices, dtype=int)
        self.cone_indices = self.cone_indices.astype(int)
        print("CONES PRE REMOVAL: ", self.cones)
        print("CONES TO REMOVE: ", cone_indices)
        keep_mask = ~np.isin(self.cone_indices, cone_indices)
        print("CONES MASK: ", keep_mask)
        print("self.cone_indices:", self.cone_indices)
        print("cone_indices to remove:", cone_indices)

        self.cones = self.cones[keep_mask]
        self.cone_indices = self.cone_indices[keep_mask]

        print("NEW CONES: ", self.cones)
        print("NEW CONE INDICES: ", self.cone_indices)
    def get_cones(self):
        """
        Returns the cones to add to SLAM Graph
        n x 3 array of n cones, x, y, color
        """
        return self.cones

    def update_mwekf_to_global_map(self, global_map):
        """
        Assign all -1 map index cones their proper map index.
        If a global map index is already assigned to another cone, remove that old cone.
        """
        print("UPDATING GLOBAL MAP:")
        print("GLOBAL MAP:", global_map)
        print("CONE INDICES:", self.cone_indices)
        print("CONES:", self.cones)

        masked_indices = np.where(self.cone_indices == -1)[0]
        cones_to_update = self.cones[masked_indices]

        # Track indices to remove (in self.cones and self.cone_indices)
        to_remove = []

        for i, cone in zip(masked_indices, cones_to_update):
            distances = np.linalg.norm(global_map[:, 0:2] - cone[0:2], axis=1)
            closest_idx = np.argmin(distances)

            # Check if this global map index is already used
            if closest_idx in self.cone_indices:
                existing_idx = np.where(self.cone_indices == closest_idx)[0][0]
                print(f"Will remove cone at index {existing_idx} already using global idx {closest_idx}")
                to_remove.append(existing_idx)

            # Assign the new map index
            self.cone_indices[i] = closest_idx

        # Remove duplicates **after** updating to avoid invalidating indices
        if to_remove:
            self.cones = np.delete(self.cones, to_remove, axis=0)
            self.cone_indices = np.delete(self.cone_indices, to_remove)

        print("MWEKF CONES:", self.cones)
        print("MWEKF INDICES:", self.cone_indices)
