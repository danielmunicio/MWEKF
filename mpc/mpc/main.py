# MPC ROS Node

# General Imports
import numpy as np
from all_settings.all_settings import MPCSettings as settings
from .mpc_controller import MPCPathFollower
from time import perf_counter, sleep

# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseArray, Pose, Point, Quaternion
from feb_msgs.msg import State, FebPath, Map
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import WheelSpeeds
import scipy as sp


def makepose(x):
    p = Pose()
    p.position.x = x[0]
    p.position.y = x[1]
    p.position.z = 0*x[3] # velocity
    p.orientation.w = np.cos(x[2]/2)
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = np.sin(x[2]/2)
    return p
class MPC(Node):
    def __init__(self):
        super().__init__('mpc_node')
        # Subscribers
        self.curr_steer_sub = self.create_subscription(Float64, '/ground_truth/wheel_speeds', self.steer_callback, 1)
        self.curr_acc_sub = self.create_subscription(Float64, '/odometry/wss', self.acc_callback, 1) 
        self.global_path_sub = self.create_subscription(FebPath, '/path/global', self.global_path_callback, 1)
        self.local_path_sub = self.create_subscription(FebPath, '/path/local', self.local_path_callback, 1)
        self.state_sub = self.create_subscription(State, '/slam/state', self.state_callback, 1)
        
        # Publishers
        self.throttle_pub = self.create_publisher(Float64, '/control/throttle', 1)
        self.steer_pub = self.create_publisher(Float64, '/control/steer', 1)
        self.control_pub = self.create_publisher(AckermannDriveStamped, 'cmd', 1)
        self.viz_pub = self.create_publisher(PoseArray, '/mpc/viz', 1)
        print('set ros subscribers and publishers')

        self.mpc = MPCPathFollower(**settings)
        self.mpc.construct_solver(use_c=True) # load compiled solver
        print("inited mpc")

        self.race_done = False
        self.path = None
        self.global_path = None
        self.local_path = None
        self.prev_soln = np.array([0.0, 0.0])
        self.DT = settings.DT/10.0
        self.curr_state = np.zeros(5)
        self.create_timer(self.DT, self.run_control)
        self.create_subscription(Bool, '/end_race', self.end_race_callback, 1)

    def end_race_callback(self, msg):
        self.race_done = True
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.acceleration = -100.0  
        msg.drive.steering_angle = self.curr_state[4]

        # with open("sim_data.txt", "a") as f:
        #     print("------------------------------------------------", file=f)
        #     print("FROM MPC:", file=f)
        #     print("Sending Acceleration Of:", msg.drive.acceleration, file=f)
        #     print("Sending Steering Of:", msg.drive.steering_angle, file=f)
        #     print("-------------------------------------------------", file=f)
            
        self.control_pub.publish(msg)

        throttle_msg = Float64()
        steer_msg = Float64()
        throttle_msg.data = -100.0
        steer_msg.data = self.curr_state[4]

        for _ in range(10):
            self.throttle_pub.publish(throttle_msg)
            self.steer_pub.publish(steer_msg)
            self.control_pub.publish(msg)

            sleep(0.1)
        rclpy.shutdown()


    def steer_callback(self, msg: WheelSpeeds):
        '''
        Return Steering Angle from steering angle sensor on car
        '''
        self.curr_steer = float(msg.steer)
    
    def acc_callback(self, msg: Float64):
        '''
        Set curr_acc to value recieved from 
        '''
        self.curr_acc = float(msg.data)

    def global_path_callback(self, msg: FebPath):
        '''
        Input: msg.PathState -> List of State (from State.msg) vectors
        Returns: List of numpy state vectors (len 4: x, y, velocity, heading)
        Description: Takes in a local or global path depending on what 
        lap we're in and converts to np.array of state vectors
        '''
        x = np.array(msg.x)
        y = np.array(msg.y)
        v = np.array(msg.v)
        psi = np.array(msg.psi)
        th = np.array(msg.th)
        a = np.array(msg.a)
        thdot = np.array(msg.thdot)
        path = np.column_stack((x, y, psi, v, th, a, thdot))
        self.global_path = path
        self.path = self.global_path if self.global_path is not None else self.local_path
        # self.update_term_cost_mats()

    def update_term_cost_mats(self):
        tic = perf_counter()
        self.Ps = []
        for waypt in self.path:
            try:
                self.Ps.append(sp.linalg.solve_continuous_are(
                    a = np.array(self.mpc.A(waypt[0:4], waypt[4:6])), 
                    b = np.array(self.mpc.B(waypt[0:4], waypt[4:6])), 
                    q = self.mpc.Q, 
                    r = self.mpc.R,
                )/self.mpc.DT)
            except np.linalg.LinAlgError:
                self.Ps.append(None) # use default of driving fwd at 10m/s
        toc = perf_counter()
        print(f"solved care {len(self.path)} times in {toc-tic:.3f}s (avg {(toc-tic)/len(self.path):.5f}s each)")

    def local_path_callback(self, msg: FebPath):
        '''
        Input: msg.PathState -> List of State (from State.msg) vectors
        Returns: List of numpy state vectors (len 4: x, y, velocity, heading)
        Description: Takes in a local or global path depending on what 
        lap we're in and converts to np.array of state vectors
        '''
        x = np.array(msg.x)
        y = np.array(msg.y)
        psi = np.array(msg.psi)
        v = np.array(msg.v)
        th = np.array(msg.th)
        a = np.array(msg.a)
        thdot = np.array(msg.thdot)
        path = np.column_stack((x, y, psi, v, th, a, thdot))
        self.local_path = path
        self.path = self.global_path if self.global_path is not None else self.local_path
        # self.update_term_cost_mats()

    def state_callback(self, carstate: State):
        '''
        Input: Float64[4]
        Description: Converts State msg to numpy vector, 
            updates variables initalized in __init__ (e.g. self.z_ref, self.z_curr, etc.) with get_update_dict and self.update,
            and solves mpc problem -> stores result in self.prev_soln
        '''
        # returns the current state as an np array with these values in this order: x,y,velocity,heading
        #curr_state = msg.carstate
        self.curr_state[0] = carstate.x # x value
        self.curr_state[1] = carstate.y # y value
        self.curr_state[2] = carstate.heading
        self.curr_state[3] = carstate.velocity
        self.curr_state[4] = carstate.theta
    
    def run_control(self):
        if self.race_done: return
        curr_state = self.curr_state
        
        pt = np.array(curr_state[0:2])
        self.curr_state[4] += self.DT*self.prev_soln[1]
        if self.path is not None: 
            idx = np.argmin(np.linalg.norm(self.path[:, :2] - pt, axis=1))
            idxs = (np.arange(idx, idx+10*self.mpc.N, 10))
            xidxs = ((idxs+10)%len(self.path)).tolist()
            uidxs = ((idxs)%len(self.path)).tolist()
            # trajectory = self.path[idxs]
            x_traj = self.path[xidxs, 0:5]
            u_traj = self.path[uidxs, 5:7]
            trajectory = np.hstack([x_traj, u_traj]).T

            self.prev_soln = self.mpc.solve(np.array(curr_state), self.prev_soln, trajectory).flatten()
            if not settings.PUBLISH: 
                # print("skipping!")
                return 
            # else: print("publishing!")

            # print(self.prev_soln)
            # print('drive message:', self.prev_soln['u_control'][0])
            # print('steering message:', self.prev_soln['u_control'][1])
            # Publishing controls
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.acceleration = self.prev_soln[0]  
            msg.drive.steering_angle = self.curr_state[4]

            # with open("sim_data.txt", "a") as f:
            #     print("------------------------------------------------", file=f)
            #     print("FROM MPC:", file=f)
            #     print("Sending Acceleration Of:", msg.drive.acceleration, file=f)
            #     print("Sending Steering Of:", msg.drive.steering_angle, file=f)
            #     print("-------------------------------------------------", file=f)
                
            self.control_pub.publish(msg)

            throttle_msg = Float64()
            steer_msg = Float64()
            throttle_msg.data = self.prev_soln[0]
            steer_msg.data = self.curr_state[4]

            self.throttle_pub.publish(throttle_msg)
            self.steer_pub.publish(steer_msg)
            msg = PoseArray()
            msg.poses = []
            msg.poses += [makepose(x) for x in self.mpc.soln[0:4, :].T]
            msg.poses += [makepose(x) for x in trajectory[0:4, :].T]
            # print(trajectory)
            # print(self.mpc.soln)

            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            self.viz_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPC()
    rclpy.spin(mpc_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
