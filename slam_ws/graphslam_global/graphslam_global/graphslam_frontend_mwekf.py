import numpy as np
from graphslamrs import GraphSLAMSolve
from all_settings.all_settings import GraphSLAMSolverSettings as solver_settings
from all_settings.all_settings import GraphSLAMSettings as settings
from all_settings.all_settings import MWEKFSettings as mwekf_settings
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
from .utility_functions import compute_timediff, quat_to_euler, compute_delta_velocity, cartesian_to_polar, rotate_and_translate_cones
from .ground_truth_publisher import Ground_Truth_Publisher
from .mwekf_backend import MWEKF_Backend

from feb_msgs.msg import State, FebPath, Map, Cones, ConesCartesian
from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from ackermann_msgs.msg import AckermannDriveStamped


class GraphSLAM_MWEKF(Node):
    def __init__(self):
        # ROS2 INTEGRATIONS
        super().__init__('graphslam_global_node')
        
        self.slam = GraphSLAMSolve(**solver_settings)
        self.mwekf = MWEKF_Backend(self, np.array([0., 0., 0., 0.]))
        self.global_map = None
        self.last_slam_update = np.array([0., 0.])
        # SUBSCRIBERS
        # SIMULATOR SPECIFIC SUBSCRIBERS 
        if (settings.using_simulator):
            if (settings.using_wheelspeeds):
                if (settings.using_ground_truth_wheelspeeds):
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ground_truth/wheel_speeds', self.wheelspeed_sub_sim, 1)
                else: 
                    self.wheelspeeds_sub = self.create_subscription(WheelSpeedsStamped, '/ros_can/wheel_speeds', 1)
            #self.state_subby = self.create_subscription(CarState, '/ground_truth/state', self.state_sub, 1,)
            self.camera_sub = self.create_subscription(ConesCartesian, '/camera/cones', self.camera_callback, 1)
        else: 
            self.wheelspeeds = self.create_subscription(Float64, '/odometry/wheelspeeds', self.wheelspeed_sub, 1)
            self.realsense_d435i_sub = self.create_subscription(ConesCartesian, '/realsense/d435i/cones', self.d435i_cones_callback, 1)
            self.realsense_d435_sub = self.create_subscription(ConesCartesian, '/realsense/d435/cones', self.d435_cones_callback, 1)


        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)
        self.cones_lidar_sub = self.create_subscription(ConesCartesian, '/lidar/cones', self.lidar_callback, 1)
        self.mpc_sub = self.create_subscription(AckermannDriveStamped, '/cmd', self.mpc_callback, 1)
        # PUBLISHERS
        self.state_pub = self.create_publisher(State, '/slam/state', 1)
        self.global_map_pub = self.create_publisher(Map, '/slam/map/global', 1)
        self.local_map_pub = self.create_publisher(Map, '/slam/map/local', 1)

        if (settings.publish_to_rviz):
            self.cones_vis_pub = self.create_publisher(PointCloud, '/slam/conemap', 1)
            self.positionguess = self.create_publisher(PointCloud, '/slam/guessed_positions', 1)
            self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 1)

        self.get_logger().info("Initialized le MWEKF")
        self.last_mpc_time = perf_counter()
        self.time = time.time()

    def mpc_callback(self, msg: AckermannDriveStamped):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        self.mwekf.last_u[:] = [msg.drive.acceleration, msg.drive.steering_angle, perf_counter()]
        self.last_mpc_time = perf_counter()

    def wheelspeed_sub_sim(self, msg: WheelSpeedsStamped):
        self.mwekf.update(np.array([((msg.speeds.lb_speed + msg.speeds.rb_speed)/2)*np.pi*0.505/60]), 3)
        self.publish_pose()

    def wheelspeed_sub(self, msg: Float64): 
        self.mwekf.update(msg.data, 3)
        self.publish_pose()

    def imu_callback(self, imu: Imu) -> None:
        if settings.using_simulator:
            self.mwekf.update(np.array([quat_to_euler(imu.orientation)]), 2)
        else: 
            forward = settings.imu_foward_direction
    
    def d435i_cones_callback(self, msg: ConesCartesian):
        self.cones_callback(rotate_and_translate_cones(msg, 'd435i'))

    def d435_cones_callback(self, msg: ConesCartesian):
        self.camera_callback(rotate_and_translate_cones(msg, 'd435'))

    def camera_callback(self, msg: ConesCartesian):
        # If we haven't created a map yet, create one 
        if self.global_map is None:
            self.global_map = np.vstack([msg.x, msg.y, msg.color]).T
            # make n x 4 array of (idx, x, y, color)
            mwekf_cones = np.vstack([np.arange(len(msg.x)), msg.x, msg.y, msg.color]).T
            self.mwekf.add_cones(mwekf_cones)
            print("FIRST UPDATE DONE")
            return
        print("GLOBAL MAP? ", self.global_map)
        print("CURRENT CONES: ", self.mwekf.get_cones())
        print("CONE INDICES: ", self.mwekf.cone_indices)
        # Returns matched_cones in tuple of form 
        # (index_in_cone_message, index in global map)
        # New cones is list of indices in cone message
        matched_cones, new_cones = self.data_association(msg)

        # Cones to send to MWEKF in form: 
        # (map_idx, cone_x, cone_y)
        mwekf_measurement_cones = []
        mwekf_new_cones = []

        for map_idx, msg_idx, in matched_cones:
            # Check which cones are in window
            # Not actually how you check if the cone is in global map but will fix later
            if map_idx in self.mwekf.cone_indices:
                # If this is in window, add it as measurement
                mwekf_measurement_cones.append((map_idx, msg.x[msg_idx], msg.y[msg_idx]))
            else: 
                # If it's not in the window, but in the global map, we add it to the window
                mwekf_new_cones.append((map_idx, msg.x[msg_idx], msg.y[msg_idx], msg.color[msg_idx]))

        for idx in new_cones:
            # Cones with -1 as their map index have NOT been added to the SLAM map
            mwekf_new_cones.append((-1, msg.x[idx], msg.y[idx], msg.color[idx]))

        # Send cones we have in our MWEKF as measurements
        if len(mwekf_measurement_cones) > 0:
            self.mwekf.update(np.array([mwekf_measurement_cones]), 1)

        # Add new cones to mwekf, and SLAM map
        if len(mwekf_new_cones) > 0:
            print("NEW CONES LLENGTH: ", len(mwekf_new_cones))
            print("------------------")
            print("LOADING TO SLAM!")
            print('---------------------')
            self.mwekf.add_cones(np.array(mwekf_new_cones))
            self.load_mwekf_to_slam()

        # Get cones behind the car and remove them
        passed_cones_indices = self.get_behind_cones()
        if len(passed_cones_indices) > 0:
            print("REMOVING BEHIND CONES: ", passed_cones_indices)
            self.mwekf.remove_cones(np.array(passed_cones_indices))


    def lidar_callback(self, msg: ConesCartesian):
        # NOTE: Thinking of doing LiDAR cannot initialize new cones
        #matched_cones, new_cones = self.data_association(msg)
        #self.camera_callback(msg)
        pass

    def data_association(self, msg: ConesCartesian):
        pos = self.mwekf.state[0:4].flatten()  # [x, y, velocity, heading]
        R = np.array([
            [np.cos(pos[3]), -np.sin(pos[3])],
            [np.sin(pos[3]),  np.cos(pos[3])]
        ])
        
        cones_message_local = np.stack([msg.x, msg.y], axis=1)
        cones_message_global = cones_message_local @ R.T + pos[:2]
        cones_message_color = np.array(msg.color)

        print("COMPARING WITH GLOBAL MAP: ", self.global_map[:, :2])
        map_pos = self.global_map[:, :2]
        map_colors = self.global_map[:, 2].astype(int)

        matched_cones = []
        new_cones = []

        for i in range(cones_message_global.shape[0]):
            message_pos = cones_message_global[i]
            message_color = cones_message_color[i]

            diffs = map_pos - message_pos
            dists = np.linalg.norm(diffs, axis=1)
            mask1 = (dists < solver_settings.max_landmark_distance)
            mask2 = (map_colors == message_color)
            mask = mask1
            if np.any(mask):
                # Only consider distances where mask is True
                map_index = np.argmin(dists[mask])
                # Recover the true index in the global map
                map_index = np.where(mask)[0][map_index]
                matched_cones.append((map_index, i))
            else:
                new_cones.append(i)

        return matched_cones, new_cones

    

    def publish_pose(self):
        state = State()
        state.x = float(self.mwekf.state[0])
        state.y = float(self.mwekf.state[1])
        state.velocity = float(self.mwekf.state[2])
        state.heading = float(self.mwekf.state[3])
        self.state_pub.publish(state)

        if settings.publish_to_rviz:
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = float(state.x)
            pose_msg.pose.position.y = float(state.y)
            pose_msg.pose.position.z = 0.
            pose_msg.pose.orientation.w = np.cos(state.heading/2)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = np.sin(state.heading/2)
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(pose_msg)

    def load_mwekf_to_slam(self):
        print("------------------------------------")
        print("LOADING TO SLAM")
        print("-------------------------------------")
        print("MWEKF STATE")
        pos = self.mwekf.state[0:2].flatten()
        cones = self.mwekf.get_cones() # n x 3 of x, y, color
        print("CONES: ", cones)
        cone_deltas = cones[:, 0:2] - pos
        dx = pos.flatten() - self.last_slam_update
        print("DX: ", dx)
        print("CONE DELTAS: ", cone_deltas)
        print("COLORS: ", cones[:, 2].astype(int))
        idxs = self.slam.update_graph(dx, cone_deltas, cones[:, 2].astype(int))

        self.last_slam_update = pos
        self.slam.solve_graph()

        # NOTE: We SHOULD do data association differently based on the fact that we 
        #       already have some cones matched to the global map, but for now we will
        #       add them all as if we have no matching
        lm_guess = np.hstack((np.array(self.slam.get_cones(indices = idxs)), cones[:, 2].reshape(-1, 1)))
        x_guess = np.array(self.slam.get_positions()[-1]).reshape(-1, 1)

        print("OLD POSE: ", pos)
        print("NEW POSE: ", x_guess)

        print("OLD MAP: ", self.global_map)
        print("NEW MAP: ", lm_guess)
        self.mwekf.state[0] = x_guess.flatten()[0]
        self.mwekf.state[1] = x_guess.flatten()[1]

        self.global_map = np.array(lm_guess)
        self.mwekf.update_global_map(lm_guess)
        self.mwekf.update((np.array(x_guess), lm_guess), 4)

    def get_behind_cones(self):
        """
        returns the indices of the cones that are behind the car
        """
        state = self.mwekf.state.flatten()[0:4]
        heading_vec = np.array([np.cos(state[3]), np.sin(state[3])])
        # Get vector from car to cones 
        cone_vectors = self.mwekf.get_cones()[:, :2] - np.array([state[0], state[1]])
        print("CONES: ", self.mwekf.get_cones())
        print("CONES LENGTH: ", len(self.mwekf.get_cones()[:, 0]))
        # Take dot product wrt car heading
        dot_prod = np.dot(cone_vectors, heading_vec)
        # Cones that are behind have negative dot product
        return np.where(dot_prod < 0)[0]

