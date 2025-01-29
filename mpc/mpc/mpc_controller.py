# XY Nonlinear Kinematic MPC Module.

# Imports
import time
import casadi as ca
import numpy as np
from .utils import discrete_dynamics, continuous_dynamics_fixed_x_order
import scipy as sp
import control as ct

class MPCPathFollower:
    def __init__(self,
                 N,
                 DT,
                 L_F,
                 L_R,
                 V_MIN,
                 V_MAX,
                 A_MIN,
                 A_MAX,
                 A_DOT_MIN,
                 A_DOT_MAX,
                 DF_MIN,
                 DF_MAX,
                 DF_DOT_MIN,
                 DF_DOT_MAX,
                 Q,
                 R,
                 **kwargs):
        self.__dict__.update(kwargs)
        for key in list(locals()):
            if key == 'self':
                pass
            elif key in 'QR': 
                setattr(self, key, ca.diag(locals()[key]))
            else:
                setattr(self, key, locals()[key])

        self.q = ca.SX.sym('q', 6, self.N)
        self.x = self.q[0:4, :]
        self.u = self.q[4:6, :]

        self.p = ca.SX.sym('p', 6, self.N)
        self.x0 = self.p[0:4, 0:1]
        self.u_prev = self.p[4:6, 0:1]
        self.xbar = self.p[0:4, 1:self.N] # target states
        self.ubar = self.p[4:6, 1:self.N] # target controls (applied one before states)
        self.P = ca.SX.sym('P', 4, 4)
        self.warmstart = dict()

        self.F, self.f, self.A, self.B = self.make_dynamics(n=3)
        
        
        A = np.array(self.A([0, 0, 0, 10], [0, 0]))
        B = np.array(self.B([0, 0, 0, 10], [0, 0]))
        print(np.linalg.matrix_rank(ct.ctrb(A, B)))



        self.default_P = sp.linalg.solve_continuous_are(
            a = A, 
            b = B, 
            q = self.Q, 
            r = self.R,
        )/self.DT

        dynamics_constr = self.x[:, 1:] - self.F.map(self.N-1)(self.x[:, :-1], self.u[:, :-1])

        self.g = []
        self.lbg = []
        self.ubg = []
        self.equality = []

        def constrain(expr, lb, ub):
            assert expr.shape==lb.shape, "lower bound must have same shape as expression"
            assert expr.shape==ub.shape, "upper bound must have same shape as expression"
            self.g.append(ca.vec(expr))
            self.lbg.append(ca.vec(lb))
            self.ubg.append(ca.vec(ub))
            self.equality.append(lb==ub)

        self.u_full = ca.horzcat(self.u_prev, self.u[:, :-1])
        cost = 0
        for stage in range(self.N):
            if stage < self.N-1:
                constrain(dynamics_constr[:, stage], ca.DM([0.0]*4), ca.DM([0.0]*4))
                # constrain((self.u_full[0, stage]-self.u_full[0, stage+1])/self.DT, ca.DM([self.A_DOT_MIN]), ca.DM([self.A_DOT_MAX]))
                # constrain((self.u_full[1, stage]-self.u_full[1, stage+1])/self.DT, ca.DM([self.DF_DOT_MIN]), ca.DM([self.DF_DOT_MAX]))
                constrain(self.u[0, stage], ca.DM([self.A_MIN]), ca.DM([self.A_MAX]))
                constrain(self.u[1, stage], ca.DM([self.DF_MIN]), ca.DM([self.DF_MAX]))
            # if stage<self.N-1: # only n-1 dynamics constraints and controls
            # if stage>0:
            if stage==0: # gap closing constraints
                constrain(self.x[:, 0]-self.x0, ca.DM([0.0]*4), ca.DM([0.0]*4))
                # constrain(self.u[:, 0]-self.u_prev, ca.DM([0.0]*2), ca.DM([0.0]*2))

            # now add costs!
            if 0<stage:
                cost += ca.bilin(self.R, self.u[:, stage]-self.ubar[:, stage-1])
            if 0<stage<self.N-1:
                cost += ca.bilin(self.Q, self.x[:, stage]-self.xbar[:, stage-1])
        cost += ca.bilin(self.P, self.x[:, self.N-1]-self.xbar[:, -1])

        nlp = {
            'x': ca.vec(self.q),
            'f': cost,
            'g': ca.vertcat(*self.g),
            'p': ca.vertcat(ca.vec(self.p), ca.vec(self.P)),
        }

        self.options = dict(**self.nlpsolver.opts, equality=np.array(ca.vertcat(*self.equality)).flatten().astype(bool).tolist())
        print(self.options)
        self.solver = ca.nlpsol('solver', self.nlpsolver.name, nlp, self.options)
        self.lbg = ca.vertcat(*self.lbg)
        self.ubg = ca.vertcat(*self.ubg)
    def solve(self, x0, u_prev, trajectory, P=None):
        print(x0.shape, u_prev.shape, trajectory.shape)
        if P is None: P = self.default_P
        # p = ca.vertcat(ca.DM(x0.reshape((4, 1))), ca.DM(u_prev.reshape((2, 1))))
        p = ca.blockcat([[ca.DM(x0.reshape((4, 1))), trajectory[0:4, :-1]], 
                         [u_prev.reshape((2, 1)), trajectory[4:6, :-1]]])
        P = ca.DM(P)
        p = ca.vertcat(ca.vec(p), ca.vec(P))
        print("warmstart:")
        print(self.warmstart)
        print("p:")
        print(p)
        print("lbg, ubg:")
        print(self.lbg, self.ubg)
        res = self.solver(p=p, lbg=self.lbg, ubg=self.ubg, **self.warmstart)
        self.warmstart = {
            'x0': res['x'],
            'lam_x0': res['lam_x'],
            'lam_g0': res['lam_g'],
        }
        self.soln = np.array(ca.reshape(res['x'], (6, self.N)))
        return self.soln[4:6, 1:2]


    def make_dynamics(self, n):
        x0 = ca.SX.sym('q0', 4)
        u0 = ca.SX.sym('u0', 2)
        x1 = continuous_dynamics_fixed_x_order(x0, u0, self.L_R, self.L_F)
        f = ca.Function('f', [x0, u0], [x1])
        A = ca.Function('A', [x0, u0], [ca.jacobian(x1, x0)])
        B = ca.Function('A', [x0, u0], [ca.jacobian(x1, u0)])
        x = x0
        for i in range(n):
            xm = x + f(x, u0)*(self.DT/(2*n))
            x = x+f(xm, u0)*(self.DT/n)

        # x0 = ca.vertcat(x0, ca.SX.sym('u0_2', 2))
        # x = ca.vertcat(x0, u0)
        
        return ca.Function('F', [x0, u0], [x]), f, A, B
class KinMPCPathFollower():
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
                setattr(self, key, ca.diag(locals()[key]))
            else:
                setattr(self, key, locals()[key])

        self.TRACK_SLACK_WEIGHT = 5e5
        self.use_rk_dynamics = False
        self.solver = 'ipopt'
        self.opti = ca.Opti()

        self.global_path = None
        self.local_path = None
        self.curr_steer = 0
        self.curr_acc = 0
        self.path = self.global_path if self.global_path is not None else self.local_path
        # self.path = np.array([[1.0, 0.0]*100])


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
        self.fix_angle = ca.Function('fix_angle', [x:=ca.MX.sym("x", 4)], [ca.horzcat(x[0, :], x[1, :], ca.sin(x[2, :]/2), x[3, :])])

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
            # 'sb': 'yes',
        } 
        if self.solver == 'worhp': self.s_opts = dict() # idk what the worhp options are
        self.opti.solver(self.solver, self.p_opts, self.s_opts)

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
                beta = ca.atan( self.L_R / (self.L_F + self.L_R) * ca.tan(self.df_dv[i]) )
                self.opti.subject_to( self.x_dv[i+1]   == self.x_dv[i]   + self.DT * (self.v_dv[i] * ca.cos(self.psi_dv[i] + beta)) )
                self.opti.subject_to( self.y_dv[i+1]   == self.y_dv[i]   + self.DT * (self.v_dv[i] * ca.sin(self.psi_dv[i] + beta)) )
                self.opti.subject_to( self.psi_dv[i+1] == self.psi_dv[i] + self.DT * (self.v_dv[i] / self.L_R * ca.sin(beta)) )
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
        constraint = self.TRACK_CON_FUN(ca.horzcat(self.x_dv[1:], self.y_dv[1:]).T)

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
            return ca.mtimes(z, ca.mtimes(Q, z.T))
        # ca.MX.sym('m').inv()
        cost = 0
        dx = ca.vertsplit(ca.diff(self.z_ref, 1, 1)[:, :2])
        dx.append(dx[-1])
        # print(dx)
        # print(self.z_ref.shape)
        # print(self.N, len(dx))
        for i in range(self.N):
            segment = dx[i]/ca.norm_2(dx[i])
            # print(segment)
            a, c, b, d = segment[0], segment[1], -segment[1], segment[0]
            # segment[0]  -segment[-1]
            # segment[1]   segment[0]
            det = a*d-b*c
            mat = ca.reshape(ca.horzcat(d, -b, 0, 0, 
                                               -c,  a, 0, 0, 
                                                0,  0, det, 0, 
                                                0,  0, 0, det), (4, 4)).T/det
            
            # 1. Error between current state and reference state for each planned timestep
            # cost += _quad_form((mat @ (self.z_dv[i+1, :]-self.z_ref[i, :]).T).T, self.Q)
            # cost += _quad_form(self.z_dv[i+1, :] - self.z_ref[i,:], 9*self.Q/(i+9)) # tracking cost
            # cost += _quad_form(self.fix_angle(self.z_dv[i+1, :] - self.z_ref[i,:]), self.Q) # tracking cost
            cost += _quad_form(self.fix_angle(self.z_dv[i+1, :] - self.z_ref[i,:]), self.Q) # tracking cost

        # 2. Error between current state and reference state for last planned timestep
        # cost += _quad_form(self.z_dv[-1, :] - self.z_ref[-1, :], self.F)

        for i in range(self.N - 1):
            # 3. Use of controls
            cost += _quad_form(self.u_dv[i+1, :] - self.u_dv[i,:], self.R)  # input derivative cost
        
        # slack costs for soft constraints
        cost += (ca.sum1(self.sl_df_dv)
               + ca.sum1(self.sl_acc_dv)
               + ca.sum1(self.sl_tr_dv)*self.TRACK_SLACK_WEIGHT)

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
