from os.path import dirname, join
from casadi import *
import numpy as np
from time import perf_counter
from dynamics import discrete_custom_integrator
import os
# hsl checking fuckery
# only works on mac/linux. if you havee windows, I'm willing to bet you have bigger problems.
hsl_avail = (
    np.any(['hsl' in j for j in sum([os.listdir(i) for i in os.environ['LD_LIBRARY_PATH'].split(":") if len(i)>1], start=[])])
 or np.any(['hsl' in j for j in sum([os.listdir(i) for i in os.environ['DYLD_LIBRARY_PATH'].split(":") if len(i)>1], start=[])])
)

assert hsl_avail, "You must have HSL linear solvers installed on your system, but they were not found (or you have windows). If you're on windows, comment out this line."

class CompiledGlobalOpt:
    DEFAULT_SOPTS = {
        'ipopt': {
            'ipopt.linear_solver': 'ma57',
            'expand': True,
            # 'ipopt.print_level': 0,
            # 'ipopt.sb': 'yes',
            # 'ipopt.ma57_automatic_scaling': 'no',
            # 'print_time': 0,
        },
        'worhp': { # https://casadi.sourceforge.net/v1.9.0/api/html/dc/ddf/classCasADi_1_1WorhpSolver.html
            'expand': True,
            'worhp.NLPprint': -1,
        },
        'snopt': {
            'expand': False,
            # 'start': "warm"
        },
    }
    def __init__(self, N, nlp_solver='ipopt', solver_opts=None, car_params={'l_r': 1.4987, 'l_f':1.5213, 'm': 1.}, bbox={'l': 2, 'w': 1}, **constraints):
        """initializes a CompiledGlobalOpt object.

        Args:
            N (int): number of points to expect in the path.
            nlp_solver (str, optional): which NLP solver plugin to use. can be 'ipopt', 'worhp', 'snopt', or whatever you have installed. Defaults to 'ipopt'.
            solver_opts (dict, optional): options to pass to NLP solver. if None, use default options. Defaults to None.
            car_params (dict, optional): parameters for dynamics model (currently bicycle model). Defaults to {'l_r': 1.4987, 'l_f':1.5213, 'm': 1.}.
            bbox (dict, optional): length and width of car (assumes car is roughly rectangular). Defaults to {'l': 2, 'w': 1}.
            DF_MAX (float, optional): maximum steering angle (radians). Defaults to 0.5.
            ACC_MIN (float, optional): maximum braking acceleration (m/s^2, should be <0). Defaults to -3.0.
            ACC_MAX_FN (float or casadi.Function, optional): function of velocity (m/s) which gives maximum forward acceleration (m/s^2) at that velocity, or constant max. Should be torque at that velocity divided by mass of car (ie, no aero consideration). Defaults to 2.0.
            V_MIN (float, optional): minimum velocity. Defaults to 0.0.
            V_MAX (float, optional): maximum velocity. Defaults to 25.0.
            FRIC_MAX (float or casadi.Function, optional): function of velocity (m/s) which gives maximum frictional acceleration (m/s^2) at that velocity, or constant max. Defaults to 12.0.
        """
        self.N = N
        self.car_params = car_params
        self.bbox = bbox
        self.nlp_solver = nlp_solver
        self.sopts = solver_opts if solver_opts else CompiledGlobalOpt.DEFAULT_SOPTS[nlp_solver]
        self.linear_solver = 'MA97' if self.nlp_solver == 'worhp' else self.sopts['ipopt.linear_solver'] if 'ipopt.linear_solver' in self.sopts else 'MUMPS'
        
        ####* constraints ####
        self.DF_MAX  =  0.5
        self.ACC_MIN = -3.0
        self.ACC_MAX_FN = 2.0
        self.DF_DOT_MAX =  0.5
        self.V_MIN = 0.0
        self.V_MAX = 25.0
        self.FRIC_MAX = 12.0

        # update with user provided constraints
        self.__dict__.update(constraints)

        # if not Function objects already, make the acceleration limiting functions 
        v = SX.sym('v')
        if not isinstance(self.FRIC_MAX, Function):
            self.FRIC_MAX = Function('FRIC_MAX', [v], [self.FRIC_MAX])
        if not isinstance(self.ACC_MAX_FN, Function):
            self.ACC_MAX_FN = Function('ACC_MAX_FN', [v], [self.ACC_MAX_FN])

        ####* utility functions ####
        # easy way to account for angle wrapping
        x = MX.sym('x', 4)
        self.fix_angle = Function('fix_angle', [x], [horzcat(x[0, :], x[1, :], sin(x[2, :]), x[3, :])])
        # generate 2x2 rotation matrices
        psi = MX.sym('psi')
        self.rot = Function('rot', [psi], [reshape(horzcat(cos(psi), sin(psi), -sin(psi), cos(psi)), 2, 2)])

        ###########################################*
        ####* Symbolic Variable Initialization ####*
        ###########################################*

        #* Track parameters
        self.bound_pairs = MX.sym('bound_pairs', self.N, 4)

        #* opt variables
        self.x = MX.sym('x', self.N, 10)
        self.t = self.x[:, 0]
        self.psi = self.x[:, 1]
        self.v = self.x[:, 2]
        self.u = self.x[:, 3:5]
        self.dt = self.x[:, 5:6]
        self.sl_dyn = self.x[:, 6:10]

        #* this represents the full state at each discretization point
        self.z = horzcat(
            self.bound_pairs[:, :2]*(1-self.t)+self.bound_pairs[:, 2:4]*self.t, # LERP between pairs of points on track bounds
            self.psi,
            self.v
        )
        
        #* this is the same thing but shifted by one index. Allows us to take diffs.
        self.z_i = vertcat(
            self.z[self.N-1:, :],
            self.z[:self.N-1, :]
        )
        ######################*
        ####* Constraints ####*
        ######################*

        #* get discrete dynamics function
        self.dynamics = discrete_custom_integrator(n=3, **car_params)
        #* these will store our constraint expressions (g) and bounds (lbg, ubg)
        self.g = []
        self.lbg = []
        self.ubg = []
        #* table to keep track of indices of each constraint
        self.gtable = dict()
        self.glen = 0

        #* Dynamics constraints: # Math: F(z_i, u_i, \Delta t_i) == z_{i+1}
        for i in range(-1, self.N-1):
            self._add_constraint(
                f'dynamics{i}',
                g = vec(self.fix_angle(
                    self.dynamics(self.z[i, :], self.u[i, :], self.dt[i, :]).T-self.z[i+1, :]) + self.sl_dyn[i, :]),
                lbg = DM([0.0]*4),
                ubg = DM([0.0]*4)
            )
        #* other constraints. all follow the same pattern.
        self._add_constraint(
            'vel',
            g = vec(self.v),
            lbg = DM([self.V_MIN]*self.N),
            ubg = DM([self.V_MAX]*self.N)
        )
        self._add_constraint(
            'acc_min',
            g = vec(self.u[:, 0]),
            lbg = DM([self.ACC_MIN]*self.N),
            ubg = DM([inf]*self.N)
        )
        self._add_constraint(
            'acc_max',
            g = vec(self.u[:, 0]-self.ACC_MAX_FN(self.v)),
            lbg = DM([-inf]*self.N),
            ubg = DM([0.0]*self.N)
        )
        self._add_constraint(
            'steering',
            g = vec(self.u[:, 1]),
            lbg = DM([-self.DF_MAX]*self.N),
            ubg = DM([self.DF_MAX]*self.N)
        )
        self._add_constraint(
            'dt',
            g = vec(self.dt),
            lbg = DM([0.0]*self.N),
            ubg = DM([1.0]*self.N)
        )
        self._add_constraint(
            'df_dot',
            g = vec(self.dt * (self.u[:, 1] - vertcat(self.u[-1:, 1], self.u[:-1, 1]))),
            lbg = DM([-self.DF_DOT_MAX]*self.N),
            ubg = DM([self.DF_DOT_MAX]*self.N)
        )
        self._add_constraint(
            't',
            g = vec(self.t),
            lbg = DM([0.0]*self.N),
            ubg = DM([1.0]*self.N)
        )
        # centripetal acceleration: # Math: \frac{v^2}{r}=\frac{v^2}{\frac{v}{\dot{\theta}}}=\frac{v^2\dot{\theta}}{v}=v\dot{\theta}
        ac = (self.v**2 / self.car_params['l_r']) * sin(arctan(self.car_params['l_r']/(self.car_params['l_f'] + self.car_params['l_r']) * tan(self.u[:, 1])))
        self._add_constraint(
            'centripetal_acc',
            g = ac**2 + self.u[:, 0]**2-self.FRIC_MAX(self.v)**2,
            lbg = DM([-inf]*self.N),
            ubg = DM([0.0]*self.N)
        )

        #* Cone Constraints: stop the car from hitting any cones
        # TODO: allow the cone locations to be different from the track side points

        #* construct a function which is close enough to a rectangle
        self.safe = Function('safespace', [x:=MX.sym('x', 2)], [(DM([1/self.bbox['w'], 1/self.bbox['l']])**6).T@x**6])

        self.cones = vertcat(self.bound_pairs[:, :2], self.bound_pairs[:, 2:4]).T
        left = self.bound_pairs[:, :2].T
        right = self.bound_pairs[:, 2:4].T
        self.nc = 5 #* number of cones to consider (ahead of and behind the current cone, on each side)
        for i in range(self.N):
            if i<self.nc:
                considered = horzcat(left[:, ((i-self.nc)%self.N):self.N], left[:, :i+self.nc],
                                     right[:, ((i-self.nc)%self.N):self.N], right[:, :i+self.nc])
            elif i+self.nc >=self.N:
                considered = horzcat(left[:, i-self.nc:self.N], left[:, :((i+self.nc)%self.N)],
                                     right[:, i-self.nc:self.N], right[:, :((i+self.nc)%self.N)])
            else:
                considered = horzcat(left[:, i-self.nc:i+self.nc],
                                     right[:, i-self.nc:i+self.nc])

            self._add_constraint(
                f'cones{i}',
                g=self.safe(self.rot(-self.psi[i])@((considered-self.z[i, :2].T) + vertcat(self.car_params['l_f']-self.car_params['l_r'], 0))).T, 
                lbg=DM([1]*self.nc*4),
                ubg=DM([inf]*self.nc*4)
            )


        #* YAY we're done adding constraints. Now concatenate all the constraints into one big vector.
        #* note that while vertcat is typically slower than horzcat, since these are all vectors, it's the exact same memory operation
        self.g = vertcat(*self.g)
        self.lbg = vertcat(*self.lbg)
        self.ubg = vertcat(*self.ubg)

        #* COST FUNCITON: # Math: \sum_{i=1}^N\left[ \Delta t_i + 10^5\cdot\sum(\text{sl}_\text{dyn})^2\right]
        #* slack vars aren't really neccessary for dynamics but I'm too lazy to remove them. Really they should be on the cone constraints but it's ok.
        self.f = sum1(self.dt) + 1e5*sumsqr(self.sl_dyn)

        #* construct the NLP dictionary to be passed into casadi.nlpsol
        self.nlp = {
            'x': vec(self.x),
            'f': self.f,
            'g': self.g,
            'p': self.bound_pairs,
        }

    def _add_constraint(self, name, g, lbg, ubg):
        """utility funciton to add a constraint and keep track of all the indices

        Args:
            name (str): name for this constraint (human-readable)
            g (casadi symbolic expression): expression to be constrained
            lbg (float): lower bound for g
            ubg (float): upper bound for g
        """
        self.gtable[name] = (self.glen, self.glen+g.shape[0])
        self.glen += g.shape[0]
        self.g.append(g)
        self.lbg.append(lbg)
        self.ubg.append(ubg)

    def construct_solver(self, generate_c=False, compile_c=False, use_c=False, gcc_opt_flag='-Ofast'):
        """creates a solver object

        Args:
            generate_c (bool, optional): whether or not to generate new C code. Defaults to False.
            compile_c (bool, optional): whether or not to look for and compile the C code. Defaults to False.
            use_c (bool, optional): whether or not to load the compiled C code. Defaults to False.
            gcc_opt_flag (str, optional): optimization flags to pass to GCC. can be -O1, -O2, -O3, or -Ofast depending on how long you're willing to wait. Defaults to '-Ofast'.
        """
        path = join(dirname(__file__), 'global_opt')
        self.solver = nlpsol('solver', self.nlp_solver, self.nlp, self.sopts)
        if generate_c: self.solver.generate_dependencies(f'{path}.c')
        if compile_c:  
            os.system(f'gcc -fPIC {gcc_opt_flag} -shared {path}.c -o {path}.so')
            # os.system(f'mv global_opt.so MPC/bin/global_opt.so') #TODO: fix this path if necessary
        if use_c:
            new_opts = self.sopts
            new_opts['expand']=False
            self.solver = nlpsol('solver', self.nlp_solver, f'{path}.so', new_opts)
    def load_solver(self):
        """alternative to construct_solver if you're just loading a saved solver.
        """
        new_opts = self.sopts
        new_opts['expand']=False
        self.solver = nlpsol('solver', self.nlp_solver, join(dirname(__file__), 'global_opt.so'), new_opts)

    def angle(self, a, b):
        cosine = (a@b)/(np.linalg.norm(a)*np.linalg.norm(b))
        cosine = 1 if cosine > 1 else -1 if cosine < -1 else cosine
        return np.arccos(cosine)*np.sign(np.linalg.det([a, b]))
    
    def to_constant_tgrid(self, dt, z, u, t):
        """converts a solver result to a constant-dt representation. z, u, and t can be passed in with **res (from solve())

        Args:
            dt (float): desired resultant time discretization interval
            z (np.ndarray): states from solver
            u (np.ndarray): controls from solver
            t (np.ndarray): timestamps from solver

        Returns:
            (np.ndarray, np.ndarray): arrays of states, controls
        """
        cur = 0.
        states = []
        controls = []
        while cur<t[-1]:
            idx = int(np.sum(t<=cur)-1)
            extra = cur-t[idx]
            controls.append(u[idx])
            states.append(np.array(self.dynamics(z[idx], u[idx], extra)).flatten())
            cur += dt
        return np.array(states), np.array(controls)

        
    def solve(self, left, right):
        """crunch the numbers.

        Args:
            left (np.ndarray): location of left cones. shape (N, 2).
            right (np.ndarray): location of right cones. shape (N, 2).

        Returns:
            dict: result of solve. keys 'z' (states), 'u' (controls), 't' (timestamps)
        """
        center = (left+right)/2
        diffs = np.diff(center, prepend=center[-1:], axis=0)
        diffs = np.concatenate([diffs, [[1, 0]]], axis=0)
        d_angles = [self.angle(diffs[i-1], diffs[i]) for i in range(1, len(diffs))]
        angles = np.cumsum(d_angles)
        
        self.x0 = vec(vertcat(
            DM([0.5]*self.N), # t
            DM(angles),       # psi
            DM([1.0]*self.N), # v
            DM([0.0]*self.N), # a
            DM([0.0]*self.N),
            DM([0.5]*self.N), # dt
            DM([0.0]*self.N*4)
        ))
        self.solver.print_options()
        self.soln = self.solver(
            x0=self.x0,
            lbg=self.lbg,
            ubg=self.ubg,
            p=horzcat(DM(left), DM(right)),
        )
        self.soln['x'] = np.array(reshape(self.soln['x'], (self.N, 10)))
        self.soln['xy'] = (left.T*(1-self.soln['x'][:, 0])+right.T*self.soln['x'][:, 0]).T

        res=dict()
        res['z'] = np.hstack([
            self.soln['xy'],
            self.soln['x'][:, 1:3],
        ])
        # print('silly:',res['z'].shape)
        res['z'] = np.concatenate([res['z'], res['z'][:1]], axis=0)
        res['u'] = np.concatenate([self.soln['x'][:, 3:5], self.soln['x'][:1, 3:5]], axis=0)
        res['t'] = np.concatenate([[0.], np.cumsum(self.soln['x'][:, 5])])
        
        return res