# XY Nonlinear Kinematic MPC Module.

# General Imports
import time
import casadi
import numpy as np
from .controller import Controller
from .utils import discrete_dynamics
from .utils import get_update_dict
from all_settings.all_settings import MPCSettings as settings
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import WheelSpeeds

# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .triangles_track_constraints import get_g_triangulation

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from feb_msgs.msg import State, FebPath

class KinMPCPathFollower(Controller, Node):
    def __init__(self, 
                 N          = 10,     # timesteps in MPC Horizon
                 DT         = 0.2,    # discretization time between timesteps (s)
                 L_F        = 1.5213, # distance from CoG to front axle (m)
                 L_R        = 1.4987, # distance from CoG to rear axle (m)
                 V_MIN      = 0.0,    # min/max velocity constraint (m/s)
                 V_MAX      = 25.0,     
                 A_MIN      = -3.0,   # min/max acceleration constraint (m/s^2)
                 A_MAX      =  2.0,     
                 DF_MIN     = -0.5,   # min/max front steer angle constraint (rad)
                 DF_MAX     =  0.5,    
                 A_DOT_MIN  = -1.5,   # min/max jerk constraint (m/s^3)
                 A_DOT_MAX  =  1.5,
                 DF_DOT_MIN = -0.5,   # min/max front steer angle rate constraint (rad/s)
                 DF_DOT_MAX =  0.5, 
                 Q = [1., 1., 10., 0.1], # weights on x, y, psi, and v.
                 R = [10., 100.],        # weights on *change in* drive and steering
                 F = [0., 0., 0., 10.],  # final state weights
                 **kwargs):
        '''
        Initializes KinMPCPathFollower object

        Arguments: see above
        '''

        # Go through all arguments and set to class attributes
        self.__dict__.update(kwargs)
        for key in list(locals()):
            if key == 'self':
                pass
            elif key in 'QRF': 
                setattr(self, key, casadi.diag(locals()[key]))
            else:
                setattr(self, key, locals()[key])

        self.TRACK_SLACK_WEIGHT = 5e5
        self.use_rk_dynamics = False
        self.solver = 'ipopt'
        self.opti = casadi.Opti()

        self.global_path = None
        self.local_path = None
        self.curr_steer = 0
        self.curr_acc = 0
        self.path = self.global_path if self.global_path is not None else self.local_path
        # self.path = np.array([[1.0, 0.0]*100])

        ### START - ROS Integration Code ###
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
        self.pointcloud_pub = self.create_publisher(PointCloud, '/mpc/viz', 1)

        ### END - ROS Integration Code ###

        ''' 
        (1) Parameters
        '''        
        self.u_prev  = self.opti.parameter(2) # previous input: [u_{acc, -1}, u_{df, -1}]
        self.z_curr  = self.opti.parameter(4) # current state:  [x_0, y_0, psi_0, v_0]

        # Reference trajectory we would like to follow.
        # First index corresponds to our desired state at timestep k+1:
        #   i.e. z_ref[0,:] = z_{desired, 1}.
        # Second index selects the state element from [x_k, y_k, psi_k, v_k].
        self.z_ref = self.opti.parameter(self.N, 4)

        self.x_ref   = self.z_ref[:,0]
        self.y_ref   = self.z_ref[:,1]
        self.psi_ref = self.z_ref[:,2]
        self.v_ref   = self.z_ref[:,3]

        '''
        (2) Decision Variables
        '''
        # Actual trajectory we will follow given the optimal solution.
        # First index is the timestep k, i.e. self.z_dv[0,:] is z_0.        
        # It has self.N+1 timesteps since we go from z_0, ..., z_self.N.
        # Second index is the state element, as detailed below.
        self.z_dv = self.opti.variable(self.N+1, 4)
    
        self.x_dv   = self.z_dv[:, 0]
        self.y_dv   = self.z_dv[:, 1]
        self.psi_dv = self.z_dv[:, 2]
        self.v_dv   = self.z_dv[:, 3]

        # Control inputs used to achieve self.z_dv according to dynamics.
        # First index is the timestep k, i.e. self.u_dv[0,:] is u_0.
        # Second index is the input element as detailed below.
        self.u_dv = self.opti.variable(self.N, 2)

        self.acc_dv = self.u_dv[:,0]
        self.df_dv  = self.u_dv[:,1]

        # Slack variables used to relax input rate and track constraints.
        # Matches self.u_dv in structure but timesteps range from -1, ..., N-1.
        # then another row for track constraints.
        self.sl_dv  = self.opti.variable(self.N , 3)
        
        self.sl_acc_dv = self.sl_dv[:,0]
        self.sl_df_dv  = self.sl_dv[:,1]
        self.sl_tr_dv  = self.sl_dv[:,2]

        self.constraint_added = False

        # Keep track of previous solution for updates
        self.prev_soln = None
        
        '''
        (3) Problem Setup: Constraints, Cost, Initial Solve
        '''
        self._add_constraints()
        
        self._add_cost()
        
        self._update_initial_condition(0., 0., 0., 1.)
        
        self._update_reference([self.DT * (x+1) for x in range(self.N)],
                              self.N*[0.], 
                              self.N*[0.], 
                              self.N*[1.])
        
        self._update_previous_input(0., 0.)
        
        # Ipopt with custom options: https://web.casadi.org/docs/ -> see sec 9.1 on Opti stack.
        # idk where to get info on worhp options
        self.p_opts = {
            'expand': True, # can only expand to sx if not using interpolant 
            # 'print_time': 0,
        }
        self.s_opts = {
            'max_cpu_time': 0.05, 
            'linear_solver': 'MA27', 
            'print_level': 0,
            # 'sb': 'yes',
        } 
        if self.solver == 'worhp': self.s_opts = dict() # idk what the worhp options are
        self.opti.solver(self.solver, self.p_opts, self.s_opts)

# ---------------------------------------------------------------------------------------------- #
        
    ### START - ROS Callback Functions ###
        
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
        path = np.column_stack((x, y, v, psi))
        self.global_path = path
        self.path = self.global_path if self.global_path is not None else self.local_path
        if self.constraint_added: return
        self.constraint_added = True

        left = [(6.11, 2.8), (6.109119415283203, 2.77589750289917), (8.934219360351562, 3.0638973712921143), (12.868500709533691, 3.012697458267212), (16.50389862060547, 3.4246973991394043), (20.136499404907227, 4.427197456359863), (22.62929916381836, 5.538097381591797), (25.49173927307129, 6.740597248077393), (28.138639450073242, 6.202697277069092), (30.521839141845703, 5.081397533416748), (32.50313949584961, 3.3362975120544434), (33.89194107055664, 1.401397466659546), (34.47583770751953, -1.504062533378601), (33.998939514160156, -5.023182392120361), (32.99224090576172, -8.059272766113281), (32.09803771972656, -10.299802780151367), (30.182140350341797, -13.539742469787598), (27.282838821411133, -15.564172744750977), (24.113739013671875, -17.043882369995117), (21.25337028503418, -18.83869171142578), (18.06159019470215, -20.45490264892578), (14.420599937438965, -22.263202667236328), (10.502239227294922, -24.330303192138672), (7.251099586486816, -26.4215030670166), (3.651329517364502, -27.45490264892578), (0.7883395552635193, -26.945703506469727), (-1.4627604484558105, -25.224702835083008), (-3.2429604530334473, -23.575801849365234), (-5.143360614776611, -21.38580322265625), (-5.6176605224609375, -17.860652923583984), (-5.938460350036621, -15.4548921585083), (-5.556060314178467, -12.859972953796387), (-5.120860576629639, -10.299802780151367), (-4.784360408782959, -7.257342338562012), (-4.843460559844971, -4.028892517089844), (-3.7221603393554688, -0.7304625511169434), (-2.1866605281829834, 1.4138973951339722), (0.24443955719470978, 2.190197467803955)]
        right = [(6.08, -1.85), (6.089759349822998, -1.8552625179290771), (8.982789993286133, -1.796582579612732), (12.939980506896973, -1.9238924980163574), (16.51483917236328, -1.6030025482177734), (20.14760971069336, -0.3732425570487976), (23.430938720703125, 0.49919745326042175), (25.965240478515625, 1.5015974044799805), (27.96493911743164, 0.9834974408149719), (29.605138778686523, -1.1794525384902954), (29.706039428710938, -4.192082405090332), (28.87574005126953, -6.820742607116699), (28.10573959350586, -8.797102928161621), (26.676239013671875, -10.299802780151367), (25.072938919067383, -11.731022834777832), (22.419269561767578, -13.14719295501709), (20.26255989074707, -14.240602493286133), (17.651329040527344, -15.4548921585083), (14.787479400634766, -16.96703338623047), (11.020049095153809, -18.86309242248535), (7.818509578704834, -20.855302810668945), (5.429309368133545, -22.412302017211914), (3.2358596324920654, -23.007902145385742), (0.9659395813941956, -21.971603393554688), (-0.9300604462623596, -19.282712936401367), (-1.157760500907898, -16.07309341430664), (-1.0045604705810547, -12.92978286743164), (-0.6089604496955872, -10.299802780151367), (-0.3480604290962219, -7.242682456970215), (-0.34576043486595154, -4.511722564697266), (0.7813395857810974, -2.6917724609375)]
        left = np.array([list(i) for i in left])
        right = np.array([list(i) for i in right])
        tr_con_fun = get_g_triangulation(left, right, [13, -10], n_pts=40, n_meshes=1, sink=0.02)
        constraint = tr_con_fun(casadi.horzcat(self.x_dv[1:], self.y_dv[1:]).T)
        # self.opti.subject_to(constraint-self.sl_tr_dv.T < 0 )
    
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
        path = np.column_stack((x, y, psi, v))
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
        
        if self.path is not None: 
            prev_controls = np.array([self.curr_steer, self.curr_acc][::-1])
            new_values = get_update_dict(pose=np.array(curr_state), prev_u=prev_controls, kmpc=self, states=self.path, prev_soln=self.prev_soln)


            self.update(new_values)
            self.prev_soln = self.solve()

            # print('drive message:', self.prev_soln['u_control'][0])
            # print('steering message:', self.prev_soln['u_control'][1])
            # Publishing controls
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.acceleration = self.prev_soln['u_control'][0]  
            msg.drive.steering_angle = self.prev_soln['u_control'][1]

            # with open("sim_data.txt", "a") as f:
            #     print("------------------------------------------------", file=f)
            #     print("FROM MPC:", file=f)
            #     print("Sending Acceleration Of:", msg.drive.acceleration, file=f)
            #     print("Sending Steering Of:", msg.drive.steering_angle, file=f)
            #     print("-------------------------------------------------", file=f)
                
            self.control_pub.publish(msg)

            throttle_msg = Float64()
            steer_msg = Float64()
            throttle_msg.data = self.prev_soln['u_control'][0]
            steer_msg.data = self.prev_soln['u_control'][1]

            self.throttle_pub.publish(throttle_msg)
            self.steer_pub.publish(steer_msg)
            self.prev_soln['u_mpc']
            pc_msg = PointCloud()
            pts = []
            
            for x in np.array(self.prev_soln['z_mpc']):
                pts.append(Point32())
                pts[-1].x = x[0]
                pts[-1].y = x[1]
                pts[-1].z = 0.0

            for x in np.vstack([new_values['x_ref'], new_values['y_ref']]).T:
                pts.append(Point32())
                pts[-1].x = x[0]
                pts[-1].y = x[1]
                pts[-1].z = 0.0
            pc_msg.points = pts
            pc_msg.header.frame_id = "map"
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            self.pointcloud_pub.publish(pc_msg)

    ### END - ROS Callback Functions ###

# ---------------------------------------------------------------------------------------------- #
        
    ### START - Update Functions to update relevant class variables ###
    
    def update(self, update_dict):
        '''
        Input: 
        - update_dict: all mpc variables to be updated 
        Description: calls _update_initial_condition, _update_reference, and _update_previous_input to actually update
        '''
        self._update_initial_condition( *[update_dict[key] for key in ['x0', 'y0', 'psi0', 'v0']] )
        self._update_reference( *[update_dict[key] for key in ['x_ref', 'y_ref', 'psi_ref', 'v_ref']] )
        self._update_previous_input( *[update_dict[key] for key in ['acc_prev', 'df_prev']] )

        if 'warm_start' in update_dict.keys():
            # Warm Start used if provided.  Else I believe the problem is solved from scratch with initial values of 0.
            self.opti.set_initial(self.z_dv,  update_dict['warm_start']['z_ws'])
            self.opti.set_initial(self.u_dv,  update_dict['warm_start']['u_ws'])
            self.opti.set_initial(self.sl_dv, update_dict['warm_start']['sl_ws'])

    def _update_initial_condition(self, x0, y0, psi0, vel0):
        '''
        Inputs:
        - x0: current x
        - y0: current y
        - psi0: current heading
        - vel0: current velocity

        Updates current state (z_curr)
        '''
        self.opti.set_value(self.z_curr, [x0, y0, psi0, vel0])

    def _update_reference(self, x_ref, y_ref, psi_ref, v_ref):
        '''
        Inputs: reference = where car should be
        - x_ref: reference x
        - y_ref: reference y
        - psi_ref: reference heading
        - v_ref: reference velocity
        '''
        self.opti.set_value(self.x_ref,   x_ref)
        self.opti.set_value(self.y_ref,   y_ref)
        self.opti.set_value(self.psi_ref, psi_ref)
        self.opti.set_value(self.v_ref,   v_ref)

    def _update_previous_input(self, acc_prev, df_prev):
        '''
        Inputs: previous = last solve
        - acc_prev: previous acceleration value
        - df_prev: previous steering angle value
        '''
        self.opti.set_value(self.u_prev, [acc_prev, df_prev])
    
    ### END - Update Functions to update relevant class variables ###

# ---------------------------------------------------------------------------------------------- #

    ### START - MPC Main Functions (Setting Up and Solving MPC Optimization Problem) ###
        
    def _add_constraints(self):
        '''
        Description:
        -> State Bound Constraints, Input Bound Constraints, Input Rate Bound Constraints, Other Constraints: 
            Ensures variables are inside bounds set in __init__, Other Contraints ensures slack variables are >= 0
        -> Initial State Constraint: Makes sure first state of planned trajectory matches current car state
        -> State Dynamics Constraints: Ensures car can move appropriately based on Car Dynamics
            - For more on equations used, read this section: 
            https://www.notion.so/MPC-Overview-31d7d1b2a56e4e36bcdf5c195f8ec5b3?pvs=4#9b47dac35e68479f81217baee5237da8
        -> Track Constraints: Car stays inside track limits
            - For more, read: https://www.notion.so/MPC-Overview-31d7d1b2a56e4e36bcdf5c195f8ec5b3?pvs=4#566651b3c5444c9e85b89aec223f2cf8
        '''
        # State Bound Constraints
        self.opti.subject_to( self.opti.bounded(self.V_MIN, self.v_dv, self.V_MAX) )        
        
        # Initial State Constraint
        self.opti.subject_to( self.x_dv[0]   == self.z_curr[0] )
        self.opti.subject_to( self.y_dv[0]   == self.z_curr[1] )
        self.opti.subject_to( self.psi_dv[0] == self.z_curr[2] )
        self.opti.subject_to( self.v_dv[0]   == self.z_curr[3] )

        # State Dynamics Constraints
        if self.use_rk_dynamics:
            F = discrete_dynamics(self.DT, self.L_R, self.L_F)
            for i in range(self.N):
                self.opti.subject_to( self.z_dv[i+1] == F(x0=self.z_dv[i], u=self.u_dv[i])['xf'])
        else:
            for i in range(self.N):
                beta = casadi.atan( self.L_R / (self.L_F + self.L_R) * casadi.tan(self.df_dv[i]) )
                self.opti.subject_to( self.x_dv[i+1]   == self.x_dv[i]   + self.DT * (self.v_dv[i] * casadi.cos(self.psi_dv[i] + beta)) )
                self.opti.subject_to( self.y_dv[i+1]   == self.y_dv[i]   + self.DT * (self.v_dv[i] * casadi.sin(self.psi_dv[i] + beta)) )
                self.opti.subject_to( self.psi_dv[i+1] == self.psi_dv[i] + self.DT * (self.v_dv[i] / self.L_R * casadi.sin(beta)) )
                self.opti.subject_to( self.v_dv[i+1]   == self.v_dv[i]   + self.DT * (self.acc_dv[i]) )

        # Input Bound Constraints
        self.opti.subject_to( self.opti.bounded(self.A_MIN,  self.acc_dv, self.A_MAX) )
        self.opti.subject_to( self.opti.bounded(self.DF_MIN, self.df_dv,  self.DF_MAX) )

        # Input Rate Bound Constraints
        self.opti.subject_to( self.opti.bounded( self.A_DOT_MIN*self.DT -  self.sl_acc_dv[0], 
                                                 self.acc_dv[0] - self.u_prev[0],
                                                 self.A_DOT_MAX*self.DT   + self.sl_acc_dv[0]) )

        self.opti.subject_to( self.opti.bounded( self.DF_DOT_MIN*self.DT  -  self.sl_df_dv[0], 
                                                 self.df_dv[0] - self.u_prev[1],
                                                 self.DF_DOT_MAX*self.DT  + self.sl_df_dv[0]) )

        for i in range(self.N - 1):
            self.opti.subject_to( self.opti.bounded( self.A_DOT_MIN*self.DT   -  self.sl_acc_dv[i+1], 
                                                     self.acc_dv[i+1] - self.acc_dv[i],
                                                     self.A_DOT_MAX*self.DT   + self.sl_acc_dv[i+1]) )
            self.opti.subject_to( self.opti.bounded( self.DF_DOT_MIN*self.DT  -  self.sl_df_dv[i+1], 
                                                     self.df_dv[i+1]  - self.df_dv[i],
                                                     self.DF_DOT_MAX*self.DT  + self.sl_df_dv[i+1]) )
        # Other Constraints
        self.opti.subject_to( 0 <= self.sl_df_dv )
        self.opti.subject_to( 0 <= self.sl_acc_dv )
        self.opti.subject_to( 0 <= self.sl_tr_dv )
        # e.g. things like collision avoidance or lateral acceleration bounds could go here.

        # Track Constraints
        if 'TRACK_CON_FUN' not in self.__dict__: return
        constraint = self.TRACK_CON_FUN(casadi.horzcat(self.x_dv[1:], self.y_dv[1:]).T)

        self.opti.subject_to(constraint-self.sl_tr_dv.T < 0 )

    def _add_cost(self):
        '''
        Description: Our cost function penalizes 3 main things
        -> _quad_form: returns (z.T)Qz where z is vector and Q is matrix
        -> 1. Error between current state and reference state for each planned timestep
        -> 2. Error between current state and reference state for last planned timestep
        -> 3. Use of controls
        '''
        def _quad_form(z, Q):
            return casadi.mtimes(z, casadi.mtimes(Q, z.T))
        # casadi.MX.sym('m').inv()
        cost = 0
        dx = casadi.vertsplit(casadi.diff(self.z_ref, 1, 1)[:, :2])
        dx.append(dx[-1])
        # print(dx)
        # print(self.z_ref.shape)
        # print(self.N, len(dx))
        for i in range(self.N):
            segment = dx[i]/casadi.norm_2(dx[i])
            # print(segment)
            a, c, b, d = segment[0], segment[1], -segment[1], segment[0]
            # segment[0]  -segment[-1]
            # segment[1]   segment[0]
            det = a*d-b*c
            mat = casadi.reshape(casadi.horzcat(d, -b, 0, 0, 
                                               -c,  a, 0, 0, 
                                                0,  0, det, 0, 
                                                0,  0, 0, det), (4, 4)).T/det
            
            # 1. Error between current state and reference state for each planned timestep
            # cost += _quad_form((mat @ (self.z_dv[i+1, :]-self.z_ref[i, :]).T).T, self.Q)
            # cost += _quad_form(self.z_dv[i+1, :] - self.z_ref[i,:], 9*self.Q/(i+9)) # tracking cost
            cost += _quad_form(self.z_dv[i+1, :] - self.z_ref[i,:], self.Q) # tracking cost

        # 2. Error between current state and reference state for last planned timestep
        # cost += _quad_form(self.z_dv[-1, :] - self.z_ref[-1, :], self.F)

        for i in range(self.N - 1):
            # 3. Use of controls
            cost += _quad_form(self.u_dv[i+1, :] - self.u_dv[i,:], self.R)  # input derivative cost
        
        # slack costs for soft constraints
        cost += (casadi.sum1(self.sl_df_dv)
               + casadi.sum1(self.sl_acc_dv)
               + casadi.sum1(self.sl_tr_dv)*self.TRACK_SLACK_WEIGHT)

        self.opti.minimize( cost )

    def solve(self):
        '''
        Returns: sol_dict (solution variables and other data related to solve) -> see below for specific info
        Description: self.opti.solve() uses casadi to find solution to mpc problem and publishes control to apply
        '''
        st = time.time()
        try:
            sol = self.opti.solve()
            # Optimal solution.
            u_mpc  = sol.value(self.u_dv)
            z_mpc  = sol.value(self.z_dv)
            sl_mpc = sol.value(self.sl_dv)
            z_ref  = sol.value(self.z_ref)
            is_opt = True
        except:
            # Suboptimal solution (e.g. timed out).
            u_mpc  = self.opti.debug.value(self.u_dv)
            z_mpc  = self.opti.debug.value(self.z_dv)
            sl_mpc = self.opti.debug.value(self.sl_dv)
            z_ref  = self.opti.debug.value(self.z_ref)
            is_opt = False

        solve_time = time.time() - st
        
        sol_dict = {}
        sol_dict['u_control']  = u_mpc[0,:]  # control input to apply based on solution
        sol_dict['optimal']    = is_opt      # whether the solution is optimal or not
        sol_dict['solve_time'] = solve_time  # how long the solver took in seconds
        sol_dict['u_mpc']      = u_mpc       # solution inputs (N by 2, see self.u_dv above) 
        sol_dict['z_mpc']      = z_mpc       # solution states (N+1 by 4, see self.z_dv above)
        sol_dict['sl_mpc']     = sl_mpc      # solution slack vars (N by 3, see self.sl_dv above)
        sol_dict['z_ref']      = z_ref       # state reference (N by 4, see self.z_ref above)

        return sol_dict

    ### END - MPC Main Functions (Setting Up and Solving MPC Optimization Problem) ###
        
def main(args=None):
    try:
        rclpy.init(args=args)
    except:
        print("init failed")
        
    mpc_node = KinMPCPathFollower(**MPCSettings)
    try:
        rclpy.spin(mpc_node)
    except Exception as e:
        print("mpc node down")
        print("fiona and cake")
        raise e
    rclpy.shutdown()

### END - RUNNING MPC NODE ###
    
if __name__ == '__main__':
    main()
