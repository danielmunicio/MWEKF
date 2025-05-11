import numpy as np
#from graphslamrs import GraphSLAMSolve
from .GraphSLAMSolve import GraphSLAMSolve
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
        self.first_solve = True
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

        self.timer = self.create_timer(0.5, self.load_mwekf_to_slam)
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
        if self.first_solve is True:
            cones = np.stack([msg.x, msg.y, msg.color], axis=1)
            self.load_new_cones_to_slam(cones)
            self.first_solve = False
            return

        # Matched_cones in n x 3 of form (map_index, local_x, local_y, color)
        # New cones is n x 3 array of form (global_x, global_y, color)
        matched_cones, new_cones = self.data_association(msg, color=True)

        if len(matched_cones > 0):
            # Send matched cones to MWEKF
            self.mwekf.update(matched_cones, 0)

        # Put new cones in SLAM and solve
        if len(new_cones > 0):
            print("LOADING NEW CONES FROM CAMERA: ", new_cones)
            self.load_new_cones_to_slam(new_cones)

    def lidar_callback(self, msg: ConesCartesian):
        # NOTE: Thinking of doing LiDAR cannot initialize new cones
        return
        if self.slam.get_cones() is None:
            return

        matched_cones, new_cones = self.data_association(msg, color=False)
        self.mwekf.update(matched_cones)


    def data_association(self, msg: ConesCartesian, color: bool):
        pos = self.mwekf.state[0:4].flatten()  # [x, y, velocity, heading]
        R = np.array([
            [np.cos(pos[3]), -np.sin(pos[3])],
            [np.sin(pos[3]),  np.cos(pos[3])]
        ])
        cones_message_local = np.stack([msg.x, msg.y], axis=1)
        cones_message_global = cones_message_local @ R.T + pos[:2]
        cones_message_color = np.array(msg.color)

        # Get global map from SLAM 
        slam_map = np.array(self.slam.get_cones())
        map_colors = np.array(self.slam.get_colors())

        matched_cones = []
        new_cones = []

        for i in range(cones_message_global.shape[0]):
            message_pos = cones_message_global[i]
            message_pos_local = cones_message_local[i]
            message_color = cones_message_color[i]

            diffs = slam_map - message_pos
            dists = np.linalg.norm(diffs, axis=1)
            mask = (dists < solver_settings.max_landmark_distance) & (map_colors == message_color)
            if np.any(mask):
                # Only consider distances where mask is True
                dists[~mask] = np.inf
                map_index = np.argmin(dists)
                # Recover the true index in the global map
                matched_cones.append((map_index, message_pos_local[0], message_pos_local[1], message_color))
            else:
                cone = (message_pos[0], message_pos[1])
                new_cones.append((message_pos[0], message_pos[1], message_color))
        return np.array(matched_cones), np.array(new_cones)


    def load_mwekf_to_slam(self):
        #NOTE: determine best update strategy
        pos = self.mwekf.state[0:2].flatten()
        cones = self.mwekf.get_cones() # n x 3 of x, y, color
        local_cones = self.local_cones(cones, return_local_frame=True)
        cone_deltas = cones[:, 0:2] - pos
        dx = pos.flatten() - self.last_slam_update
        idxs = self.slam.update_graph(dx, cone_deltas, cones[:, 2].astype(int))

        self.last_slam_update = pos
        self.slam.solve_graph()

        print("INDICES: ", idxs)
        print("SLAM GET CONES: ", np.array(self.slam.get_cones(indices = idxs)))

        lm_guess = np.hstack((np.array(self.slam.get_cones(indices = idxs)), cones[:, 2].reshape(-1, 1)))
        self.publish_cone_map(lm_guess)

        x_guess = np.array(self.slam.get_positions()[-1]).reshape(-1, 1)

        global_solve_cones = np.array(self.slam.get_cones())
        global_solve_colors = np.array(self.slam.get_colors())

        # Coping by just forcing MWEKF states instead of sending the measurements
        self.mwekf.state[0] = x_guess.flatten()[0]
        self.mwekf.state[1] = x_guess.flatten()[1]
        self.mwekf.cones = np.array(lm_guess)
        
        mwekf_cones = self.local_cones(lm_guess, return_local_frame=True)
        new_local_cones = self.local_cones(lm_guess, return_local_frame=False)
        #self.mwekf.update(x_guess.flatten()[0:2], 4)
        #self.mwekf.update(mwekf_cones, 5)

        self.publish_cone_map(new_local_cones)

    def load_new_cones_to_slam(self, cones):
        pos = self.mwekf.state[0:2].flatten()
        
        print("LOADING NEW CONES: ", cones)
        cone_deltas = cones[:, 0:2] - pos
        dx = pos.flatten() - self.last_slam_update
        idxs = self.slam.update_graph(dx, cone_deltas, cones[:, 2].astype(int))
        print("IDXS IN LOAD NEW CONES: ", idxs)

        self.last_slam_update = pos
        self.slam.solve_graph()
        print("---------------------------")
        print("INDICES: ", idxs)
        print("SLAM GET CONES: ", np.array(self.slam.get_cones(indices = idxs)))
        print("---------------------------")
        print("ERROR ABOUT TO HAPPEn")
        # lm guess should be ONLY the new cones
        lm_guess = np.hstack((np.array(self.slam.get_cones(indices = idxs)), cones[:, 2].reshape(-1, 1)))

        # There's probably a better way to do this, but this code
        # should add all the cones that haven't been added to the mwekf map, to the mwekf map
        if self.mwekf.get_cones() is not None:
            lm_guess_len = len(lm_guess[:, 0])
            cones_len = len(cones[:, 0])
            slam_map_len = len(np.array(self.slam.get_cones())[:, 0])
            mwekf_cones_len = len(np.array(self.mwekf.get_cones()[:, 0]))

            print("LM GUESS LEN: ", lm_guess_len)
            print("SLAM MAP LEN: ", slam_map_len)
            print("MWEKF CONES LEN", mwekf_cones_len)
            print("MWEKF CONES: ", self.mwekf.get_cones())
            print("SLAM MAP: ", np.array(self.slam.get_cones()))
            assert lm_guess_len == cones_len
            assert slam_map_len - mwekf_cones_len == cones_len

        self.mwekf.add_cones(lm_guess)

    def publish_cone_map(self, lm_guess):   
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

    def local_cones(self, cones, radius=10, return_local_frame=True):
        cones_xy = cones[:, 0:2]
        state = self.mwekf.state.flatten()
        curpos = np.array([state[0], state[1]])
        heading = state[3]

        close_cones = cones_xy[((cones_xy - curpos) ** 2).sum(axis=1) < radius * radius]
        delta_vecs = close_cones - curpos

        rotation_matrix = np.array([
            [np.cos(-heading), -np.sin(-heading)],
            [np.sin(-heading),  np.cos(-heading)]
        ])
        local_coords = (rotation_matrix @ delta_vecs.T).T

        mask = local_coords[:, 0] > 0

        return local_coords[mask] if return_local_frame else close_cones[mask]