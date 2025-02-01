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
                 RUNTIME_FREQUENCY,
                 **kwargs):
        self.__dict__.update(kwargs)
        for key in list(locals()):
            if key == 'self':
                pass
            elif key in 'QR': 
                setattr(self, key, ca.diag(locals()[key]))
            else:
                setattr(self, key, locals()[key])

        self.q = ca.SX.sym('q', 8, self.N+1)
        self.x = self.q[0:6, :]
        self.u = self.q[6:8, :]

        self.p = ca.SX.sym('p', 6, self.N+1)
        self.x0 = self.p[0:4, 0:1]
        self.u_prev = self.p[4:6, 0:1]
        self.xbar = self.p[0:4, 1:self.N+1] # target states
        self.ubar = self.p[4:6, 1:self.N+1] # target controls (applied one before states)
        self.P = ca.SX.sym('P', 4, 4)
        self.warmstart = dict()

        self.F, self.f, self.A, self.B = self.make_dynamics(n=3)
        self.fix_angle = ca.Function('fix_angle', [x:=ca.MX.sym("x", 4)], [ca.horzcat(x[0, :], x[1, :], 2*ca.pi*ca.sin(x[2, :]/2), x[3, :])])

        
        
        A = np.array(self.A([0, 0, 0, 10], [0, 0]))
        B = np.array(self.B([0, 0, 0, 10], [0, 0]))
        assert 3.9<np.linalg.matrix_rank(ct.ctrb(A, B))<4.1 # it's an integer (mathematically at least)



        self.default_P = sp.linalg.solve_continuous_are(
            a = A, 
            b = B, 
            q = self.Q, 
            r = self.R,
        )/self.DT

        dynamics_constr = self.x[:, 1:] - self.F.map(self.N)(self.x[:, :-1], self.u[:, :-1])
        du = self.x[4:6, :] - self.u[:, :]

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
            self.equality.append(lb==ub)

        cost = 0
        for stage in range(self.N):
            if stage < self.N:
                constrain(dynamics_constr[:, stage], ca.DM([0.0]*6), ca.DM([0.0]*6))
                constrain(du[0:1, stage]/(self.DT if stage>0 else 1/self.RUNTIME_FREQUENCY), ca.DM([self.A_DOT_MIN]), ca.DM([self.A_DOT_MAX]))
                constrain(du[1:2, stage]/(self.DT if stage>0 else 1/self.RUNTIME_FREQUENCY), ca.DM([self.DF_DOT_MIN]), ca.DM([self.DF_DOT_MAX]))
                constrain(self.u[0:1, stage], ca.DM([self.A_MIN]), ca.DM([self.A_MAX]))
                constrain(self.u[1:2, stage], ca.DM([self.DF_MIN]), ca.DM([self.DF_MAX]))
            if stage==0:
                constrain(self.x[0:4, 0]-self.x0, ca.DM([0.0]*4), ca.DM([0.0]*4))
                constrain(self.x[4:6, 0]-self.u_prev, ca.DM([0.0]*2), ca.DM([0.0]*2))

            # now add costs!
            if stage<self.N:
                cost += ca.bilin(self.R, self.u[:, stage]-self.ubar[:, stage])
            if stage<self.N-1:
                cost += ca.bilin(self.Q, self.fix_angle(self.x[0:4, stage+1]-self.xbar[:, stage]))
        cost += ca.bilin(self.P, self.fix_angle(self.x[0:4, self.N]-self.xbar[:, self.N-1]))

        nlp = {
            'x': ca.vec(self.q),
            'f': cost,
            'g': ca.vertcat(*self.g),
            'p': ca.vertcat(ca.vec(self.p), ca.vec(self.P)),
        }

        self.options = dict(**self.nlpsolver.opts, equality=np.array(ca.vertcat(*self.equality)).flatten().astype(bool).tolist())
        self.solver = ca.nlpsol('solver', self.nlpsolver.name, nlp, self.options)
        self.lbg = ca.vertcat(*self.lbg)
        self.ubg = ca.vertcat(*self.ubg)
    def solve(self, x0, u_prev, trajectory, P=None):
        print(x0.shape, u_prev.shape, trajectory.shape)
        if P is None: P = self.default_P
        p = ca.blockcat([[ca.DM(x0.reshape((4, 1))), trajectory[0:4, :]], 
                         [u_prev.reshape((2, 1)), trajectory[4:6, :]]])
        P = ca.DM(P)
        p = ca.vertcat(ca.vec(p), ca.vec(P))

        res = self.solver(p=p, lbg=self.lbg, ubg=self.ubg, **self.warmstart)
        self.warmstart = {
            'x0': res['x'],
            'lam_x0': res['lam_x'],
            'lam_g0': res['lam_g'],
        }
        self.soln = np.array(ca.reshape(res['x'], (8, self.N+1)))
        return self.soln[6:8, 0:1]


    def make_dynamics(self, n):

        # state: [ x, y, theta, v] (theta is heading)
        # control: [a, phi] (fwd acceleration, steering angle)

        x0 = ca.SX.sym('q0', 4)
        u0 = ca.SX.sym('u0', 2)

        beta = ca.arctan(self.L_R/(self.L_F + self.L_R) * ca.tan(u0[1]))

        # calculate dx/dt
        x1 = ca.vertcat(x0[3] * ca.cos(x0[2] + beta),  # dxPos/dt = v*cos(theta+beta)
                        x0[3] * ca.sin(x0[2] + beta),  # dyPos/dt = v*sin(theta+beta)
                        x0[3] / self.L_R * ca.sin(beta),   # dtheta/dt = v/l_r*sin(beta)
                        u0[0],                    # dv/dt = a
        )
        f = ca.Function('f', [x0, u0], [x1])
        A = ca.Function('A', [x0, u0], [ca.jacobian(x1, x0)])
        B = ca.Function('A', [x0, u0], [ca.jacobian(x1, u0)])

        # n-step midpoint method (2nd-order RK, no corrector)
        x = x0
        for i in range(n):
            xm = x + f(x, u0)*(self.DT/(2*n))
            x += f(xm, u0)*(self.DT/n)

        u_prev = ca.SX.sym('u0_2', 2)
        
        return ca.Function('F', [ca.vertcat(x0, u_prev), u0], [ca.vertcat(x, u0)]), f, A, B
