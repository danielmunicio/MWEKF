# MPC ROS Node

# General Imports
import numpy as np
from .utils import get_update_dict
from all_settings.all_settings import MPCSettings as settings
from .mpc_controller import MPCPathFollower

# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from feb_msgs.msg import State, FebPath, Map
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import WheelSpeeds


class MPC(Node):
    def __init__(self):
        super().__init__('mpc_node')
        # Subscribers
        self.curr_steer_sub = self.create_subscription(Float64, '/ground_truth/wheel_speeds', self.steer_callback, 1)
        self.curr_acc_sub = self.create_subscription(Float64, '/odometry/wss', self.acc_callback, 1) 
        self.global_path_sub = self.create_subscription(FebPath, '/path/global', self.global_path_callback, 1)
        self.local_path_sub = self.create_subscription(FebPath, '/path/local', self.local_path_callback, 1)
        self.state_sub = self.create_subscription(State, '/slam/state', self.state_callback, 1)
        self.map_sub = self.create_subscription(Map, '/slam/map/local', self.map_callback, 1)
        
        # Publishers
        self.throttle_pub = self.create_publisher(Float64, '/control/throttle', 1)
        self.steer_pub = self.create_publisher(Float64, '/control/steer', 1)
        self.control_pub = self.create_publisher(AckermannDriveStamped, 'cmd', 1)
        self.pointcloud_pub = self.create_publisher(PointCloud, '/mpc/viz', 1)
        print('set ros subscribers and publishers')

        self.mpc = MPCPathFollower(**settings)
        print("inited mpc")

        self.path = None
        self.global_path = None
        self.local_path = None
        self.prev_soln = np.array([0.0, 0.0])
        self.prev_path = np.array([[0.0, 0.0]]*settings.N)
        self.cones = np.array([[10.0, 10.0]]*settings.N_CONES).T
    def map_callback(self, msg: Map):
        cones = np.array([
            list(msg.left_cones_x) + list(msg.right_cones_x),
            list(msg.left_cones_y) + list(msg.right_cones_y),
        ])

        if cones.shape[1] <= settings.N_CONES:
            self.cones = np.hstack([cones] + [cones[:, -1:]]*(settings.N_CONES - cones.shape[1]))
        else:
            self.cones = np.array(sorted(self.cones.T, key=lambda cone: np.min(np.linalg.norm(self.prev_path-cone, axis=1))))[0:settings.N_CONES].T

        
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
        a = np.array(msg.a)
        theta = np.array(msg.theta)
        path = np.column_stack((x, y, v, psi, a, theta))
        self.global_path = path
        self.path = self.global_path if self.global_path is not None else self.local_path
        # if self.constraint_added: return
        # self.constraint_added = True

        # left = [(6.11, 2.8), (6.109119415283203, 2.77589750289917), (8.934219360351562, 3.0638973712921143), (12.868500709533691, 3.012697458267212), (16.50389862060547, 3.4246973991394043), (20.136499404907227, 4.427197456359863), (22.62929916381836, 5.538097381591797), (25.49173927307129, 6.740597248077393), (28.138639450073242, 6.202697277069092), (30.521839141845703, 5.081397533416748), (32.50313949584961, 3.3362975120544434), (33.89194107055664, 1.401397466659546), (34.47583770751953, -1.504062533378601), (33.998939514160156, -5.023182392120361), (32.99224090576172, -8.059272766113281), (32.09803771972656, -10.299802780151367), (30.182140350341797, -13.539742469787598), (27.282838821411133, -15.564172744750977), (24.113739013671875, -17.043882369995117), (21.25337028503418, -18.83869171142578), (18.06159019470215, -20.45490264892578), (14.420599937438965, -22.263202667236328), (10.502239227294922, -24.330303192138672), (7.251099586486816, -26.4215030670166), (3.651329517364502, -27.45490264892578), (0.7883395552635193, -26.945703506469727), (-1.4627604484558105, -25.224702835083008), (-3.2429604530334473, -23.575801849365234), (-5.143360614776611, -21.38580322265625), (-5.6176605224609375, -17.860652923583984), (-5.938460350036621, -15.4548921585083), (-5.556060314178467, -12.859972953796387), (-5.120860576629639, -10.299802780151367), (-4.784360408782959, -7.257342338562012), (-4.843460559844971, -4.028892517089844), (-3.7221603393554688, -0.7304625511169434), (-2.1866605281829834, 1.4138973951339722), (0.24443955719470978, 2.190197467803955)]
        # right = [(6.08, -1.85), (6.089759349822998, -1.8552625179290771), (8.982789993286133, -1.796582579612732), (12.939980506896973, -1.9238924980163574), (16.51483917236328, -1.6030025482177734), (20.14760971069336, -0.3732425570487976), (23.430938720703125, 0.49919745326042175), (25.965240478515625, 1.5015974044799805), (27.96493911743164, 0.9834974408149719), (29.605138778686523, -1.1794525384902954), (29.706039428710938, -4.192082405090332), (28.87574005126953, -6.820742607116699), (28.10573959350586, -8.797102928161621), (26.676239013671875, -10.299802780151367), (25.072938919067383, -11.731022834777832), (22.419269561767578, -13.14719295501709), (20.26255989074707, -14.240602493286133), (17.651329040527344, -15.4548921585083), (14.787479400634766, -16.96703338623047), (11.020049095153809, -18.86309242248535), (7.818509578704834, -20.855302810668945), (5.429309368133545, -22.412302017211914), (3.2358596324920654, -23.007902145385742), (0.9659395813941956, -21.971603393554688), (-0.9300604462623596, -19.282712936401367), (-1.157760500907898, -16.07309341430664), (-1.0045604705810547, -12.92978286743164), (-0.6089604496955872, -10.299802780151367), (-0.3480604290962219, -7.242682456970215), (-0.34576043486595154, -4.511722564697266), (0.7813395857810974, -2.6917724609375)]
        # left = np.array([list(i) for i in left])
        # right = np.array([list(i) for i in right])
    
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
        a = np.array(msg.a)
        theta = np.array(msg.theta)
        path = np.column_stack((x, y, psi, v, a, theta))
        self.local_path = path
        self.path = self.global_path if self.global_path is not None else self.local_path

    def state_callback(self, carstate: State):
        '''
        Input: Float64[4]
        Description: Converts State msg to numpy vector, 
            updates variables initalized in __init__ (e.g. self.z_ref, self.z_curr, etc.) with get_update_dict and self.update,
            and solves mpc problem -> stores result in self.prev_soln
        '''
        # returns the current state as an np array with these values in this order: x,y,velocity,heading
        #curr_state = msg.carstate
        curr_state = [0., 0., 0., 0.]
        curr_state[0] = carstate.x # x value
        curr_state[1] = carstate.y # y value
        curr_state[2] = carstate.heading
        curr_state[3] = carstate.velocity

        pt = np.array(curr_state[0:2])
        
        if self.path is not None: 
            idx = np.argmin(np.linalg.norm(self.path[:, :2] - pt, axis=1))
            idxs = (np.arange(idx, idx+10*self.mpc.N, 10))
            xidxs = ((idxs+10)%len(self.path)).tolist()
            uidxs = ((idxs)%len(self.path)).tolist()
            # trajectory = self.path[idxs]
            x_traj = self.path[xidxs, 0:4]
            u_traj = self.path[uidxs, 4:6]
            trajectory = np.hstack([x_traj, u_traj]).T

            self.prev_soln, self.prev_path = self.mpc.solve(np.array(curr_state), self.prev_soln, trajectory, self.cones, np.eye(4))
            self.prev_soln = self.prev_soln.flatten()
            print(self.prev_soln)
            # print('drive message:', self.prev_soln['u_control'][0])
            # print('steering message:', self.prev_soln['u_control'][1])
            # Publishing controls
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.acceleration = self.prev_soln[0]  
            msg.drive.steering_angle = self.prev_soln[1]

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
            steer_msg.data = self.prev_soln[1]

            self.throttle_pub.publish(throttle_msg)
            self.steer_pub.publish(steer_msg)
            pc_msg = PointCloud()
            pts = []
            
            for x in self.mpc.soln[0:4, :].T:
                pts.append(Point32())
                pts[-1].x = x[0]
                pts[-1].y = x[1]
                pts[-1].z = 0.0

            for x in trajectory[0:4, :].T:
                pts.append(Point32())
                pts[-1].x = x[0]
                pts[-1].y = x[1]
                pts[-1].z = 0.0
            
            # for x in self.cones.T:
            #     pts.append(Point32())
            #     pts[-1].x = x[0]
            #     pts[-1].y = x[1]
            #     pts[-1].z = 2.0

            print(trajectory.shape)
            pc_msg.points = pts
            pc_msg.header.frame_id = "map"
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            self.pointcloud_pub.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPC()
    rclpy.spin(mpc_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
