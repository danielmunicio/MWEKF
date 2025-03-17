from casadi import *
import numpy as np
from time import perf_counter
from .dynamics import discrete_custom_integrator
from all_settings.all_settings import MPCSettings as mpc_settings
import os
# hsl checking fuckery
# only works on mac/linux. if you havee windows, I'm willing to bet you have bigger problems.
hsl_avail = False
paths = ":".join([(linuxpath if (linuxpath:=os.environ.get('LD_LIBRARY_PATH')) is not None else ""),
                  (macospath if (macospath:=os.environ.get('DYLD_LIBRARY_PATH')) is not None else "")])
for folder in paths.split(":"):
    if len(folder)>1 and np.any(['libhsl' in j for j in os.listdir(folder)]):
        hsl_avail = True
        break

assert hsl_avail, "You must have HSL linear solvers installed on your system, but they were not found (or you have windows). If you're on windows, comment out this line."

## MPC class variable that'll keep info abt the car + help us solve optimzation problems!
class CompiledLocalOpt:
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
        """initializes a CompiledLocalOpt object.

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
        self.sopts = solver_opts if solver_opts else CompiledLocalOpt.DEFAULT_SOPTS[nlp_solver]
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
        x = MX.sym('x', 5)
        # self.fix_angle = Function('fix_angle', [x], [horzcat(x[0, :], x[1, :], sin(2*x[2, :]), x[3, :])])
        self.fix_angle = Function('fix_angle', [x], [horzcat(x[0, :], x[1, :], sin(x[2, :]/2), x[3, :], x[4, :])])
        # generate 2x2 rotation matrices
        psi = MX.sym('psi')
        self.rot = Function('rot', [psi], [reshape(horzcat(cos(psi), sin(psi), -sin(psi), cos(psi)), 2, 2)])

        ###########################################*
        ####* Symbolic Variable Initialization ####*
        ###########################################*

        #* Track parameters
        self.bound_pairs = MX.sym('bound_pairs', self.N, 4)
        self.curr_state = MX.sym('curr_state', 1, 5)
        # self.curr_state = currAndCones[0, :]
        # self.bound_pairs = currAndCones[1:, :]

        #* opt variables
        self.x = MX.sym('x', self.N, 7)
        self.t = self.x[:, 0]
        self.psi = self.x[:, 1]
        self.v = self.x[:, 2]
        self.theta = self.x[:, 3]
        self.u = self.x[:, 4:6]
        self.dt = self.x[:, 6:7]
        # self.sl_dyn = self.x[:, 7:11]

        #* this represents the full state at each discretization point
        self.z = horzcat(
            self.bound_pairs[:, :2]*(1-self.t)+self.bound_pairs[:, 2:4]*self.t, # LERP between pairs of points on track bounds
            self.psi,
            self.v,
            self.theta,
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
        dynamics = self.fix_angle.map(self.N-1)(
                        self.dynamics.map(self.N-1, 'thread', 19)(
                            self.z[:-1, :].T, self.u[:-1, :].T, self.dt[:-1, :].T
                        ) - self.z[1:, :].T
                    )
        # for i in range(-1, self.N-1):
        #     self._add_constraint(
        #         f'dynamics{i}',
        #         g = vec(self.fix_angle(
        #             self.dynamics(self.z[i, :], self.u[i, :], self.dt[i, :]).T-self.z[i+1, :])),# + self.sl_dyn[i, :]),
        #         lbg = DM([0.0]*5),
        #         ubg = DM([0.0]*5)
        #     )
        self._add_constraint(
            'dynamics',
            vec(dynamics),
            vec(DM.zeros(dynamics.shape)),
            vec(DM.zeros(dynamics.shape))
        )

        # #* Dynamics constraints: # Math: F(z_i, u_i, \Delta t_i) == z_{i+1}
        # for i in range(0, self.N-1):
        #     # print(self.z[i, :]);
        #     self._add_constraint(
        #         f'dynamics{i}',
        #         g = vec(self.fix_angle(
        #             self.dynamics(self.z[i, :], self.u[i, :], self.dt[i, :]).T-self.z[i+1, :])),# + self.sl_dyn[i, :]),
        #         lbg = DM([0.0]*5),
        #         ubg = DM([0.0]*5)
        #     )
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
            g = vec(self.x[:, 4]),
            lbg = DM([-self.DF_MAX]*self.N),
            ubg = DM([self.DF_MAX]*self.N)
        )
        self._add_constraint(
            'dt',
            g = vec(self.dt),
            lbg = DM([0.0]*self.N),
            ubg = DM([2.0]*self.N)
        )
        self._add_constraint(
            'df_dot',
            g = vec(self.u[:, 1]),
            lbg = DM([-self.DF_DOT_MAX]*(self.N)),
            ubg = DM([self.DF_DOT_MAX]*(self.N))
        )
        self._add_constraint(
            't',
            g = vec(self.t),
            lbg = DM([0.4]*self.N),
            ubg = DM([0.6]*self.N)
        )
        self._add_constraint(
            't0',
            g = self.t[0],
            lbg = DM([0.5]),
            ubg = DM([0.5]),
        )
        self._add_constraint(
            'tf',
            g = self.t[-1],
            lbg = DM([0.5]),
            ubg = DM([0.5]),
        )
        # Keeps initial heading and velocity unchangeable
        self._add_constraint(
            'curr_velocity',
            g = self.curr_state[3] - self.v[0],
            lbg = DM(0),
            ubg = DM(0)
        )
        self._add_constraint(
            'final velocity',
            g = self.v[-1],
            lbg=DM([0.5]),
            ubg=DM([1.0]),
        )
        self._add_constraint(
            'final control',
            g = self.u[-1,:].T,
            lbg=DM([0.0, 0.0]),
            ubg=DM([0.0, 0.0]),
        )
        # self._add_constraint(
        #     'curr_heading',
        #     g = self.curr_state[2] - self.psi[0],
        #     lbg = DM(-pi/6),
        #     ubg = DM(pi/6)
        # )
        # Makes it so that the time to run is at minimum 2 (soft constraint, scalar included)
        # self.scalar = MX.sym("scalar")
        # self._add_constraint(
        #     'min_time',
        #     g = sum1(self.dt) + self.scalar,
        #     lbg = DM(0.5),
        #     ubg = DM(float('inf'))
        # )
        # centripetal acceleration: # Math: \frac{v^2}{r}=\frac{v^2}{\frac{v}{\dot{\theta}}}=\frac{v^2\dot{\theta}}{v}=v\dot{\theta}
        ac = (self.v**2 / self.car_params['l_r']) * sin(arctan(self.car_params['l_r']/(self.car_params['l_f'] + self.car_params['l_r']) * tan(self.theta)))
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
        considered = horzcat(left[:, 1:-1], right[:, 1:-1])
        for i in range(self.N):
            pass
            # self._add_constraint(
            #     f'cones{i}',
            #     g=self.safe(self.rot(-self.psi[i])@((considered-self.z[i, :2].T) + vertcat(self.car_params['l_f']-self.car_params['l_r'], 0))).T, 
            #     lbg=DM([1.0]*(self.N-2)*2),
            #     ubg=DM([inf]*(self.N-2)*2)
            # )


        #* YAY we're done adding constraints. Now concatenate all the constraints into one big vector.
        #* note that while vertcat is typically slower than horzcat, since these are all vectors, it's the exact same memory operation
        print([i.shape for i in self.g])
        self.g = vertcat(*self.g)
        self.lbg = vertcat(*self.lbg)
        self.ubg = vertcat(*self.ubg)

        #* COST FUNCITON: # Math: \sum_{i=1}^N\left[ \Delta t_i + 10^5\cdot\sum(\text{sl}_\text{dyn})^2\right]
        #* slack vars aren't really neccessary for dynamics but I'm too lazy to remove them. Really they should be on the cone constraints but it's ok.
        # adding the scalar portion will penalize having to use the scalar (i.e. giving too short a path)
        self.f = sum1(self.dt)# + 1e12*sumsqr(self.sl_dyn)**2 + 1e2*(self.scalar)**2

        #* construct the NLP dictionary to be passed into casadi.
        # make sure you add the self.scalar to x so the nlp can modify it!
        self.nlp = {
            'x': vertcat(vec(self.x)),# self.scalar),
            'f': self.f,
            'g': self.g,
            'p': vertcat(vec(self.curr_state), vec(self.bound_pairs)),
        }

        self.warmstart = None

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
        self.solver = nlpsol('solver', self.nlp_solver, self.nlp, self.sopts)
        if generate_c: 
            self.solver.generate_dependencies('local_opt.c')
            os.system(f"mv local_opt.c {os.path.dirname(__file__)}/local_opt.c")
        if compile_c:  
            os.system(f'gcc -fPIC {gcc_opt_flag} -shared {os.path.dirname(__file__)}/local_opt.c -o {os.path.dirname(__file__)}/local_opt.so')
            # os.system(f'mv local_opt.so MPC/bin/local_opt.so') #TODO: fix this path if necessary
        if use_c:
            new_opts = self.sopts
            new_opts['expand']=False
            self.solver = nlpsol('solver', self.nlp_solver, f'{os.path.dirname(__file__)}/local_opt.so', new_opts)
    def load_solver(self):
        """alternative to construct_solver if you're just loading a saved solver.
        """
        new_opts = self.sopts
        new_opts['expand']=False
        self.solver = nlpsol('solver', self.nlp_solver, 'local_opt.so', new_opts)

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
            if len(states)>1 and abs(states[-1][2] - states[-2][2])>1.5*pi:
                states[-1][2] = states[-1][-2] + (states[-1][2]-states[-2][2])%(2*pi)

            cur += dt
        # now pad it until it's 1.5x whats needed, for safety
        while len(states)<mpc_settings.N*10*2:
            controls.append(u[-2])
            controls[-1][0]=0.0

            states.append(np.array(self.dynamics(states[-1].tolist(), u[-2], dt)).flatten())

        return np.array(states), np.array(controls)

        
    def solve(self, left, right, curr_state, err_ok=True):
        """crunch the numbers.

        Args:
            left (np.ndarray): location of left cones. shape (N, 2).
            right (np.ndarray): location of right cones. shape (N, 2).
            curr_state (np.ndarray): current state of the car. shape (x, y, phi, v).

        Returns:
            dict: result of solve. keys 'z' (states), 'u' (controls), 't' (timestamps)
        """
        # print(repr(left))
        # print(repr(right))
        # print(repr(curr_state))
        center = (left+right)/2
        diffs = np.diff(center, axis=0)
        diffs = np.concatenate([[[1, 0]], diffs], axis=0)
        angles = [self.angle(diffs[0], diffs[i]) for i in range(1, len(diffs))]
        angles.append(angles[-1])
        angles = np.array(angles)
        # angles = np.cumsum(angles)

        # left = center
        # right = center

        # print(f"angles shape: {angles.shape}")
        self.x0 = self.warmstart if self.warmstart is not None else vec(vertcat(
            DM([0.5]*self.N), # t
            DM(angles),       # psi
            DM([curr_state[3]]*self.N), # v
            DM([curr_state[4]]*self.N), # th
            DM([0.0]*self.N), # a
            DM([0.0]*self.N), # thdot
            DM([0.1]*self.N), # dt
            # DM([0.0]*self.N*4),
            # DM([0.0])         # scalar
        ))
        # print(self.x0, self.x0.shape())
        # print("leo is mean!!!!!")
        # print(DM(curr_state).shape, horzcat(DM(left), DM(right)).shape)
        # print(dict(
        #     x0=self.x0,
        #     lbg=self.lbg,
        #     ubg=self.ubg,
        #     p=vertcat(DM(curr_state).T, horzcat(DM(left), DM(right))),
        # ))
        self.soln = self.solver(
            x0=self.x0,
            lbg=self.lbg,
            ubg=self.ubg,
            p=vertcat(vec(DM(curr_state).T), vec(horzcat(DM(left), DM(right)))),
        )
        # self.warmstart = self.soln['x']
        # print("SOLVE FINISHED")
        if err_ok and not self.solver.stats()['success']:
            raise AssertionError("Solver failed to converge")
        
        self.soln['x'] = np.array(reshape(self.soln['x'], (self.N, 7)))
        # print("LOCAL OPT OUTPUT")
        # print(self.soln)
        self.soln['xy'] = (left.T*(1-self.soln['x'][:, 0])+right.T*self.soln['x'][:, 0]).T
        # print(self.soln['x'][:, 5])
        res=dict()
        res['z'] = np.hstack([
            self.soln['xy'],
            self.soln['x'][:, 1:4],
        ])
        # print('silly:',res['z'].shape)
        res['u'] = self.soln['x'][:, 4:6]
        res['t'] = np.concatenate([[0.], np.cumsum(self.soln['x'][:, 6])[:-1]])
        

        ## if the solved solution doesn't fill all 2 seconds
        ### NAIVE (pad "straight" controls until 2 seconds are filled)
        # if res['t'][-1] < 2.0:
        #     step_size = res['t'][1] - res['t'][0]
        #     for i in range(res['t'][-1], 2.0 + step_size, step_size):
        #         ###### TO DO ######
        #         # res['z'] = np.append(res[''])
        #         # res['u'] = np.append(res['u'], )
        #         res['t'] = np.append(res['t'], i)
        ### IMPROVED (stretch optimized controls across the 2 seconds)
        # added constraint that the solution must be at least 2 seconds long

        return res