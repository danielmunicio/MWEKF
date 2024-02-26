# XY Nonlinear Kinematic MPC Module.

# General Imports
import time
import casadi
from controller import Controller
from utils import discrete_dynamics
from utils import get_update_dict
import numpy as np

# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from feb_msgs.msg import State
from feb_msgs.msg import FebPath

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
         
        ### ROS Integration Code ###
        super().__init__('mpc_node')

        # Subscribers
        self.curr_steer = self.create_subscription(Float64, '/odometry/steer', self.steer_callback, 1)
        self.curr_acc = self.create_subscription(Float64, '/odometry/wss', self.acc_callback, 1) 
        self.global_path = self.create_subscription(FebPath, 'path/global', self.path_callback, 1)
        self.local_path = self.create_subscription(FebPath, 'path/local', self.path_callback, 1)
        self.path = self.global_path if self.global_path is not None else self.local_path
        self.state = self.create_subscription(State, '/slam/state', self.state_callback, 1)
        
        # Publishers
        self.trottle = self.create_publisher(Float64, '/control/trottle', 1)
        self.steer = self.create_publisher(Float64, '/control/steer', 1)

        ############################

        self.TRACK_SLACK_WEIGHT = 5e5
        self.use_rk_dynamics = False
        self.solver = 'ipopt'

        self.__dict__.update(kwargs)
        for key in list(locals()):
            if key == 'self':  pass
            elif key in 'QRF': setattr(self, key, casadi.diag(locals()[key]))
            else:              setattr(self, key, locals()[key])

        self.opti = casadi.Opti()

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
            'print_time': 0,
        }
        self.s_opts = {
            'max_cpu_time': 0.05, 
            'linear_solver': 'MA27', 
            'print_level': 0,
            'sb': 'yes',
        } 
        if self.solver == 'worhp': self.s_opts = dict() # idk what the worhp options are
        self.opti.solver(self.solver, self.p_opts, self.s_opts)

        self.prev_soln = self.solve()

    ### ROS Callback Functions ###
    def steer_callback(self, msg: Float64):
        '''
        Return Steering Angle from steering angle sensor on car
        '''
        return float(msg)
    
    def acc_callback(self, msg: Float64):
        '''
        Return Acceleration from acelerometer on car
        '''
        return float(msg)

    def path_callback(self, msg: FebPath):
        '''
        Input: msg.PathState -> List of State (from State.msg) vectors
        Returns: Matrix of State vectors (4 x n)
        '''
        # TODO: Fix 
        path = []
        count = 0
        point = []
        for val in msg.PathState:
            if count >= 6:
                point = np.array(point)
                path.append(point)
                point = []
            point.append(val)
            count += 1
        path = np.array(path)
        return path

    def state_callback(self, msg: State):
        # returns the current state as an np array with these values in this order: x,y,velocity,heading
        curr_state = np.zeros((1,4), np.float64)
        curr_state[0] = msg.State[0] # x value
        curr_state[1] = msg.State[1] # y value
        curr_state[2] = msg.State[2] # velocity
        curr_state[3] = msg.State[3] # heading
        
        prev_controls = np.array([self.curr_steer, self.curr_acc])
        new_values = get_update_dict(pose=curr_state, prev_u=self.FROM_ODOMETRY, kmpc=self, states=self.path, prev_soln=self.prev_soln)
        self.update(new_values)
        self.prev_soln = self.solve()
        
    def _add_constraints(self):
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
            
            cost += _quad_form((mat@(self.z_dv[i+1, :]-self.z_ref[i, :]).T).T, self.Q)
            # cost += _quad_form(self.z_dv[i+1, :] - self.z_ref[i,:], self.Q) # tracking cost
        # cost += _quad_form(self.z_dv[-1, :] - self.z_ref[-1, :], self.F)

        for i in range(self.N - 1):
            cost += _quad_form(self.u_dv[i+1, :] - self.u_dv[i,:], self.R)  # input derivative cost
        
        # slack costs for soft constraints
        cost += (casadi.sum1(self.sl_df_dv)
               + casadi.sum1(self.sl_acc_dv)
               + casadi.sum1(self.sl_tr_dv)*self.TRACK_SLACK_WEIGHT)

        self.opti.minimize( cost )

    def solve(self):
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

    def update(self, update_dict):
        self._update_initial_condition( *[update_dict[key] for key in ['x0', 'y0', 'psi0', 'v0']] )
        self._update_reference( *[update_dict[key] for key in ['x_ref', 'y_ref', 'psi_ref', 'v_ref']] )
        self._update_previous_input( *[update_dict[key] for key in ['acc_prev', 'df_prev']] )

        if 'warm_start' in update_dict.keys():
            # Warm Start used if provided.  Else I believe the problem is solved from scratch with initial values of 0.
            self.opti.set_initial(self.z_dv,  update_dict['warm_start']['z_ws'])
            self.opti.set_initial(self.u_dv,  update_dict['warm_start']['u_ws'])
            self.opti.set_initial(self.sl_dv, update_dict['warm_start']['sl_ws'])

    def _update_initial_condition(self, x0, y0, psi0, vel0):
        self.opti.set_value(self.z_curr, [x0, y0, psi0, vel0])

    def _update_reference(self, x_ref, y_ref, psi_ref, v_ref):
        self.opti.set_value(self.x_ref,   x_ref)
        self.opti.set_value(self.y_ref,   y_ref)
        self.opti.set_value(self.psi_ref, psi_ref)
        self.opti.set_value(self.v_ref,   v_ref)

    def _update_previous_input(self, acc_prev, df_prev):
        self.opti.set_value(self.u_prev, [acc_prev, df_prev])
    
    

### RUNNING MPC NODE ###
def main(args=None):
    rclpy.init(args=args)
    mpc_node = KinMPCPathFollower()
    rclpy.spin(mpc_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
