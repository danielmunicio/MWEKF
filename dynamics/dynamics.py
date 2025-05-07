import casadi as ca
from dataclasses import dataclass
from abc import ABC, abstractmethod
from enum import Enum

class BicycleModelState(ca.MX):
    def __init__(self, var = None):
        if var is None:
            var = ca.MX.sym('q', 5)
        super().__init__(var)
        self._q = var

    @property
    def x(self): return self[0]
    @property
    def y(self): return self[1]
    @property
    def theta(self): return self[2]
    @property
    def v(self): return self[3]
    @property
    def phi(self): return self[4]
    @property
    def nx(self): return 5


class BicycleModelControl(ca.MX):
    def __init__(self, var = None):
        if var is None:
            var = ca.MX.sym('u', 2)
        super().__init__(var)
        self._u = var

    @property
    def a(self): return self[0]
    @property
    def phidot(self): return self[1]
    @property
    def nu(self): return 2

@dataclass
class BicycleModelParams:
    L_R: float
    L_F: float


class DynamicsModel(ABC):
    @property
    @abstractmethod
    def xdot(self): pass
    @property
    @abstractmethod
    def q(self): pass
    @property
    @abstractmethod
    def u(self): pass
    @property
    @abstractmethod
    def dt(self): pass
    @property
    @abstractmethod
    def f(self): pass
    @property
    @abstractmethod
    def A(self): pass
    @property
    @abstractmethod
    def B(self): pass
    @property
    @abstractmethod
    def F(self): pass
    @property
    @abstractmethod
    def A_discrete(self): pass
    @property
    @abstractmethod
    def B_discrete(self): pass
    @property
    @abstractmethod
    def affine_f(self): pass
    @property
    @abstractmethod
    def affine_g(self): pass
    @property
    @abstractmethod
    def p(self): pass

class Discretization(Enum):
    RK4 = 'rk4'
    MIDPOINT = 'midpoint'

class BicycleModel(DynamicsModel):
    def __init__(
        self, 
        p: BicycleModelParams = BicycleModelParams(0.8128, 0.8128), 
        q: BicycleModelState = None, 
        u: BicycleModelControl = None, 
        dt: ca.MatrixCommon = None, 
        discretization_method: str = Discretization.RK4, 
        n: int = 1
    ):
        self._p = p
        self._q = BicycleModelState(ca.MX.sym('q', 5)) if (q is None) else q
        self._u = BicycleModelControl(ca.MX.sym('u', 2)) if (u is None) else u
        self._dt = ca.MX.sym('dt') if dt is None else dt

        beta = ca.arctan(self._p.L_R/(self._p.L_F + self._p.L_R) * ca.tan(self._q.phi))

        # calculate dx/dt
        self._xdot = BicycleModelState(ca.vertcat(self._q.v * ca.cos(self._q.theta + beta),  # dxPos/dt = v*cos(theta+beta)
                        self._q.v * ca.sin(self._q.theta + beta),  # dyPos/dt = v*sin(theta+beta)
                        self._q.v / self._p.L_R * ca.sin(beta),   # dtheta/dt = v/l_r*sin(beta)
                        self._u.a,                    # dv/dt = a
                        self._u.phidot
        ))
        self._f = Function[BicycleModelState]('f', [self._q, self._u], [self._xdot])

        # assumes the dynamics can actually be converted to control affine form
        self._affine_f = ca.Function('f', [self._q], [ca.jacobian(ca.substitute(self._xdot, self._u, ca.DM.zeros(2)), self._q)])
        self._affine_g = ca.Function('g', [self._q], [ca.jacobian(ca.substitute(self._xdot, self._u, ca.DM.ones(2)), self._u)])

        self._A = ca.Function('A', [self._q, self._u], [ca.jacobian(self._xdot, self._q)])
        self._B = ca.Function('B', [self._q, self._u], [ca.jacobian(self._xdot, self._u)])
        
        h = self._dt/n
        x = self._q
        if discretization_method == 'midpoint':
            for _ in range(n):
                xm = x + self._f(x, self._u)*(h/2)
                x += self._f(xm, self._u)*h
        elif discretization_method == 'rk4':
            for _ in range(n):
                k1 = h*self._f(x, self._u)
                k2 = h*self._f(x+h*k1/2, self._u)
                k3 = h*self._f(x+h*k2/2, self._u)
                k4 = h*self._f(x+h*k3, self._u)
                x += (k1 + 2*k2 + 2*k3 + k4)/6
        if isinstance(self._dt, ca.MatrixCommon):
            inputs = [self._q, self._u, self._dt]
        else:
            inputs = [self._q, self._u]
        self._F = Function[BicycleModel]('F', inputs, [x])
        self._A_discrete = ca.Function('A_discrete', inputs, ca.jacobian(self._F(*inputs), self._q))
        self._B_discrete = ca.Function('B_discrete', inputs, ca.jacobian(self._F(*inputs), self._u))
    
    @property
    def xdot(self): return BicycleModelState(self.xdot)
    @property
    def q(self): return self._q
    @property
    def u(self): return self._u
    @property
    def dt(self): return self._dt
    @property
    def f(self): return self._f
    @property
    def A(self): return self._A
    @property
    def B(self): return self._B
    @property
    def F(self): return self._F
    @property
    def A_discrete(self): return self._A_discrete
    @property
    def B_discrete(self): return self._B_discrete
    @property
    def affine_f(self): return self._affine_f
    @property
    def affine_g(self): return self._affine_g
    @property
    def p(self): return self._p

class FunctionMeta(type):
    def __getitem__(self, classes):
        class FunctionWrapped(ca.Function):
            def __call__(self, *args, **kwargs):
                out = super().__call__(*args, **kwargs)
                if isinstance(classes, tuple):
                    return tuple(i[0](i[1]) for i in zip(classes, out))
                return classes(out)
        return FunctionWrapped
class Function(metaclass=FunctionMeta): pass