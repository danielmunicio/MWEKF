import casadi as ca
from casadi import MX, DM
import numpy as np

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

def discrete_dynamics(tstep, l_r=0.5, l_f=0.5, m=1.0):
    x = MX.sym('x', 4)
    u = MX.sym('u', 2)
    xdot = continuous_dynamics_fixed_x_order(x, u, l_r, l_f, m)
    ode = {
        'x': x,
        'u': u,
        'ode': xdot,
    }
    return ca.integrator('integrator', 'rk', ode, 0, tstep, {'number_of_finite_elements': 2, 'expand': True, 'simplify': True})

def closest_point(pt):
    idx = np.argmin(np.linalg.norm(states[:, :2]-pt, axis=1))
    if idx+kmpc.N>states.shape[0]:
        return np.vstack([states[idx:],states[:kmpc.N-(states.shape[0]-idx)]])
    return states[idx:idx+kmpc.N, :]

def get_update_dict(pose, prev_u, prev_soln=None):
    # pts = g.subpath_from_point(pose[:2], kmpc.N, 0.).T
    # idx = np.argmin(np.linalg.norm(path[:65, :2]-pose[:2], axis=1))
    # pts = path[idx:idx+kmpc.N]
    # pts = time_path.subpath_from_point(pose[:2], kmpc.N+1)[1:]
    pts = closest_point(pose[:2])
    update_dict = dict(
        x0 = pose[0],
        y0 = pose[1],
        psi0 = pose[2],
        v0 = pose[3],

        x_ref = pts[:, 0],
        y_ref = pts[:, 1],
        psi_ref = pts[:, 2],
        v_ref = pts[:, 3],

        acc_prev = prev_u[0],
        df_prev = prev_u[1],
    )
        ##* warm start: use values from previous solve
    if prev_soln:
        update_dict['warm_start'] = dict(
            z_ws = prev_soln['z_mpc'], # state
            u_ws = prev_soln['u_mpc'], # control
            sl_ws = prev_soln['sl_mpc'] # slack vars for soft constraints
        )
    
    return update_dict