# XY Nonlinear Kinematic MPC Module.

# Imports
import time
import casadi as ca
import numpy as np
import scipy as sp
import control as ct
import os

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
                 RUNTIME_FREQUENCY,
                 ivpsolver,
                 **kwargs):
        self.__dict__.update(kwargs)
        for key in list(locals()):
            if key == 'self':
                pass
            elif key in 'QR': 
                setattr(self, key, ca.diag(locals()[key]))
            else:
                setattr(self, key, locals()[key])

        ### initialize optimization variables
        # each column is a timestep containing [states, controls]
        # There are 6 states because we have augmented the state to
        # include the previous timestep's controls
        # this is necessary to implement jerk limits.

        # x = [x, y, psi, v, acc_prev, theta_prev]
        # u = [acc, theta]

        self.q = ca.SX.sym('q', 7, self.N+1)
        self.x = self.q[0:5, :]
        self.u = self.q[5:7, :]

        ### Initialize parameters
        # this is the target trajectory in the (non-augmented) state and controls
        # plus the initial state in the first column
        self.p = ca.SX.sym('p', 7, self.N+1)
        self.x0 = self.p[0:5, 0:1]
        self.xbar = self.p[0:5, 1:self.N+1] # target states
        self.ubar = self.p[5:7, 0:self.N] # target controls (applied one timestep before target states)

        ##### q (opt vars) if n=3
        # | t = 0         | t = 1             | t = 2             | t = 3             |
        # |---------------|-------------------|-------------------|-------------------|
        # | initial x (x0)| predicted  x 1    | predicted x 2     | predicted x     3 | (5 rows)
        # | u(0)          | u(1)              | u(2)              | (unused)          | (2 rows)

        ##### p (parameters) if n=3
        # | t = 0         | t = 1             | t = 2             | t = 3             |
        # |---------------|-------------------|-------------------|-------------------|
        # | initial x (x0)| xbar(1)           | xbar(2)           | xbar(3)           | (5 rows)
        # | ubar(0)       | ubar(1)           | ubar(2)           | (unused)          | (2 rows)


        # Additional parameter: terminal cost matrix
        # technically doesn't need to be 16 params since it's symmetric but issok
        # doesn't really impact performance
        self.P = ca.SX.sym('P', 5, 5)


        # dictionary to hold warmstart keyword arguments.
        # empty now; will be updated when we run the solver.
        self.warmstart = dict()

        # get dynamics
        # returns:
        #   - F: discrete dynamics with augmented state (evolves ([x(k), u(k-1)], u(k)) to [x(k+1), u(k)])
        #   - f: continuous dynamics with normal state (gives dx/dt from x and u)
        #   - A: jacobian of f wrt x
        #   - B: jacobian of f wrt u
        self.F, self.f, self.A, self.B = self.make_dynamics(**self.ivpsolver)

        # hack for angle wrapping
        self.fix_angle = ca.Function('fix_angle', [x:=ca.MX.sym("x", 5)], [ca.horzcat(x[0, :], x[1, :], 2*ca.pi*ca.sin(x[2, :]/2), x[3, :], x[4, :])])


        #### compute default terminal cost that assumes we're traveling forward at 10 m/s
        # first compute linearized system and check controllability
        # A = np.array(self.A([0, 0, 0, 10], [0, 0]))
        # B = np.array(self.B([0, 0, 0, 10], [0, 0]))
        # assert 3.9<np.linalg.matrix_rank(ct.ctrb(A, B))<4.1 # it's an integer (mathematically at least)
        # # then solve the CARE to get a lyapunov terminal cost
        # self.default_P = sp.linalg.solve_continuous_are(
        #     a = A, 
        #     b = B, 
        #     q = self.Q, 
        #     r = self.R,
        # )/self.DT
        self.default_P = self.Q
        # formulate dynamics constraint using map, which helps the expression graph be more compact        
        dynamics_constr = self.x[:, 1:] - self.F.map(self.N)(self.x[:, :-1], self.u[:, :-1])
        # formulate differential of u, which we can use to constrain rate of change.

        self.g = []
        self.lbg = []
        self.ubg = []
        self.equality = []

        def constrain(expr, lb, ub):
            assert expr.shape==lb.shape, f"lower bound must have same shape as expression, but got {lb.shape} and {expr.shape}"
            assert expr.shape==ub.shape, f"upper bound must have same shape as expression, but got {ub.shape} and {expr.shape}"
            self.g.append(ca.vec(expr))
            self.lbg.append(ca.vec(lb))
            self.ubg.append(ca.vec(ub))
            # we must keep track of which constraints are equalities bc fatrop needs this info
            self.equality.append(lb==ub) 

        # now we actually add all the constraints!
        # we must do this all in a big loop so that each stage's constraints are grouped together.
        # the order of the constraints here is important; we must match the structure FATROP expects.
        cost = 0
        for stage in range(self.N+1):
            if stage < self.N:
                # dynamics (gap-closing constraints)
                constrain(dynamics_constr[:, stage], ca.DM([0.0]*5), ca.DM([0.0]*5))
                # drive jerk limit
                # constrain(du[0:1, stage]*(self.RUNTIME_FREQUENCY if stage==0 else 1/self.DT), ca.DM([self.A_DOT_MIN]), ca.DM([self.A_DOT_MAX]))
                # steering velocity limit
                # constrain(du[1:2, stage]*(self.RUNTIME_FREQUENCY if stage==0 else 1/self.DT), ca.DM([self.DF_DOT_MIN]), ca.DM([self.DF_DOT_MAX]))
                # control bounds
                constrain(self.u[0:1, stage], ca.DM([self.A_MIN]), ca.DM([self.A_MAX]))
                constrain(self.u[1:2, stage], ca.DM([self.DF_DOT_MIN]), ca.DM([self.DF_DOT_MAX]))
            if stage==0:
                # initial states
                constrain(self.x[0:5, 0]-self.x0, ca.DM([0.0]*5), ca.DM([0.0]*5))
                # constrain(self.x[5:7, 0]-self.u_prev, ca.DM([0.0]*2), ca.DM([0.0]*2))
            else:
                constrain(self.x[3, stage], ca.DM([self.V_MIN]), ca.DM([self.V_MAX]))
                constrain(self.x[4, stage], ca.DM([self.DF_MIN]), ca.DM([self.DF_MAX]))

            

            if stage<self.N:
                # control cost
                cost += ca.bilin(self.R, self.u[:, stage] - self.ubar[:, stage])
            if 0<stage<self.N:
                cost += ca.bilin(self.Q, self.fix_angle(self.x[:, stage] - self.xbar[:, stage-1]).T)

        # terminal state cost
        cost += ca.bilin(self.P, self.fix_angle(self.x[:, self.N]-self.xbar[:, self.N-1]))

        # collect everything into vectors in the format for `nlpsol`
        self.nlp = {
            'x': ca.vec(self.q),
            'f': cost,
            'g': ca.vertcat(*self.g),
            'p': ca.vertcat(ca.vec(self.p), ca.vec(self.P)),
        }

        # setup options
        # we update the options dict from settings with the equality list
        # which tells the solver which constraints are equality vs inequality constraints
        if self.nlpsolver.name=='fatrop':
            self.nlpsolver.opts['equality'] = np.array(ca.vertcat(*self.equality)).flatten().astype(bool).tolist()
        self.options = dict(**(self.nlpsolver.opts))

        # now construct the solver and the constraints!
        self.solver = ca.nlpsol('solver', self.nlpsolver.name, self.nlp, self.options)
        self.lbg = ca.vertcat(*self.lbg)
        self.ubg = ca.vertcat(*self.ubg)

    def construct_solver(self, generate_c=False, compile_c=False, use_c=False, gcc_opt_flag='-Ofast'):
        """handles codegeneration and loading.

        Args:
            generate_c (bool, optional): whether or not to generate new C code. Defaults to False.
            compile_c (bool, optional): whether or not to look for and compile the C code. Defaults to False.
            use_c (bool, optional): whether or not to load the compiled C code. Defaults to False.
            gcc_opt_flag (str, optional): optimization flags to pass to GCC. can be -O1, -O2, -O3, or -Ofast depending on how long you're willing to wait. Defaults to '-Ofast'.
        """
        if generate_c: 
            self.solver.generate_dependencies('mpc.c')
            os.system(f"mv mpc.c {os.path.dirname(__file__)}/mpc.c")
        if compile_c:  
            os.system(f'gcc -fPIC {gcc_opt_flag} -shared {os.path.dirname(__file__)}/mpc.c -o {os.path.dirname(__file__)}/mpc.so')
        if use_c:
            self.solver = ca.nlpsol('solver', self.nlpsolver.name, f'{os.path.dirname(__file__)}/mpc.so', self.options)

    def solve(self, x0, u_prev, trajectory, P=None):
        """crunch the numbers; solve the problem.

        Args:
            x0: length 4 vector of the current car state.
            u_prev: length 2 vector of the previous control command
            trajectory: shape (6, N) trajectory of target states and controls.
            P (optional): terminal cost matrix. If not passed, use default from 10 m/s fwd driving.

        Returns:
            array of shape (2, 1): control result. [[acc], [theta]]
        """
        if P is None: P = self.default_P
        p = ca.blockcat([[ca.DM(x0.reshape((5, 1))), trajectory[0:5, :]], 
                         [trajectory[5:7, :], ca.DM.zeros(2, 1)]])
        P = ca.DM(P)
        p = ca.vertcat(ca.vec(p), ca.vec(P))

        res = self.solver(p=p, lbg=self.lbg, ubg=self.ubg, **self.warmstart)
        self.warmstart = {
            'x0': res['x'],
            'lam_x0': res['lam_x'],
            'lam_g0': res['lam_g'],
        }
        self.soln = np.array(ca.reshape(res['x'], self.q.shape)) # (8, self.N+1)
        return self.soln[5:7, 0:1]


    def make_dynamics(self, n, method='rk4'):
        """construct functions for the dynamics of the car. uses bicycle model.

        Args:
            n (int): number of steps the integrator should take.
            method (str, optional): integration method to use. one of ('midpoint', 'rk4'). Defaults to 'rk4' for a 4th order explicit runge-kutta method

        Returns:
            (F, f, A, B): discrete dynamics w/ augmented state, continuous dynamics, jac(f, x), jac(f, u). All casadi.Function objects.
        """

        # state: [ x, y, theta, v] (theta is heading)
        # control: [a, phi] (fwd acceleration, steering angle)

        x0 = ca.SX.sym('q0', 5)
        u0 = ca.SX.sym('u0', 2)

        beta = ca.arctan(self.L_R/(self.L_F + self.L_R) * ca.tan(x0[4]))

        # calculate dx/dt
        x1 = ca.vertcat(x0[3] * ca.cos(x0[2] + beta),  # dxPos/dt = v*cos(theta+beta)
                        x0[3] * ca.sin(x0[2] + beta),  # dyPos/dt = v*sin(theta+beta)
                        x0[3] / self.L_R * ca.sin(beta),   # dtheta/dt = v/l_r*sin(beta)
                        u0[0],                    # dv/dt = a
                        u0[1]
        )

        f = ca.Function('f', [x0, u0], [x1])
        A = ca.Function('A', [x0, u0], [ca.jacobian(x1, x0)])
        B = ca.Function('A', [x0, u0], [ca.jacobian(x1, u0)])

        if method == 'midpoint':
            x = x0
            for _ in range(n):
                xm = x + f(x, u0)*(self.DT/(2*n))
                x += f(xm, u0)*(self.DT/n)
        elif method == 'rk4':
            x = x0
            h = self.DT/n
            for _ in range(n):
                k1 = h*f(x, u0)
                k2 = h*f(x+h*k1/2, u0)
                k3 = h*f(x+h*k2/2, u0)
                k4 = h*f(x+h*k3, u0)
                x += (k1 + 2*k2 + 2*k3 + k4)/6
        
        return ca.Function('F', [x0, u0], [x]), f, A, B
