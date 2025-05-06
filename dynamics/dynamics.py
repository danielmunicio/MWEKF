import casadi as ca
import dataclasses
from dataclasses import dataclass
from abc import ABC


@dataclass
class BicycleModelState:
    _q: ca.MatrixCommon

    @property.getter
    def x(self): return self._q[0]
    @property.getter
    def y(self): return self._q[1]
    @property.getter
    def theta(self): return self._q[2]
    @property.getter
    def v(self): return self._q[3]
    @property.getter
    def phi(self): return self._q[4]
    @property.getter
    def nx(self): return self._q.shape[0]
    @property.getter
    def q(self): return self._q


@dataclass
class BicycleModelControl:
    _u: ca.MatrixCommon

    @property.getter
    def a(self): return self._u[0]
    @property.getter
    def phidot(self): return self._u[1]
    @property.getter
    def u(self): return self._u
    @property.getter
    def nu(self): return self._u.shape[0]


class BicycleModel:
    def __init__(self, x=None, u=None, t=None):
        self._x = BicycleModelState(ca.MX.sym('q', 5) if (x is None) else x)
        self._u = BicycleModelControl(ca.MX.sym('u', 2) if (u is None) else u)
        self._t = ca.SX.sym('t') if (t is None) else t
        self._f = ca.MX.sym('xdot', 5)
        self.affine_f = ca.MX.sym('xdot', 5, 5)
        self.affine_g = ca.MX.sym('xdot', 5, 2)
        
class BycicleModel(ca.Callback):
    def __init__(self, name, x, u, t, opts={}):
        super().__init__(self)
        self.x = BicycleModelState(x)
        self.u = BicycleModelControl(u)
        self.t = t
        self.construct(name, opts)

    # Number of inputs and outputs
    def get_n_in(self): return 1
    def get_n_out(self): return 1

    # Initialize the object
    def init(self):
        print('initializing object')

    # Evaluate numerically
    def eval(self, arg):
        x = arg[0]
        f = sin(self.d*x)
        return [f]