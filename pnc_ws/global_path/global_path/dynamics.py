import casadi as ca
from casadi import MX, Function, arctan, tan, sin, cos, horzcat
def continuous_dynamics_fixed_x_order(x, u, l_r=0.5, l_f=0.5, m=1.0):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,v,theta]
    input u = [F,phi]
    """
    # physical constants
    # l_r  distance rear wheels to center of gravitiy of the car
    # l_f  distance front wheels to center of gravitiy of the car
    # m    mass of the car

    # set parameters
    beta = ca.arctan(l_r/(l_f + l_r) * ca.tan(u[1]))

    # calculate dx/dt
    return ca.vertcat(x[3] * ca.cos(x[2] + beta),  # dxPos/dt = v*cos(theta+beta)
                      x[3] * ca.sin(x[2] + beta),  # dyPos/dt = v*sin(theta+beta)
                      x[3] / l_r * ca.sin(beta),   # dtheta/dt = v/l_r*sin(beta)
                      u[0] / m)                    # dv/dt = F/m
def discrete_custom_integrator(n = 3, l_r=0.5, l_f=0.5, m=1.0, compile_c=False, load=True):
    x0 = MX.sym('x0', 4)
    u = MX.sym('u', 2)
    dt = MX.sym('t')
    xdot = continuous_dynamics_fixed_x_order(x0, u, l_r, l_f, m)
    f = ca.Function('f', [x0, u], [xdot])
    # return ca.Function('integrator', [x0, u, dt], [x0+f(x0, u)*dt])
    x = x0
    for i in range(n):
        xm = x + f(x, u)*(dt/(2*n))
        x = x+f(xm, u)*(dt/n)

    return ca.Function('integrator', [x0, u, dt], [x])

