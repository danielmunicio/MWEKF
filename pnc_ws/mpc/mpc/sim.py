#%%
import casadi as ca
import numpy as np
from casadi import MX, DM
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64
import matplotlib.pyplot as plt
from sys import path
from os.path import join, dirname, abspath
path.append(join(dirname(abspath(__file__)), '..','..', 'GraphSLAMP'))
from track import get_test_track, offset_path
path.append(join(dirname(abspath(__file__)), '..', 'custom_opt'))
from global_opt import returnPoints




def f(): pass
function = f.__class__
class SimpleSim:
    def __init__(self, controller, dynamics, tstep, thoriz):
        """initializes a simulator object

        Args:
            controller (ca.Function): the controller to use. function of x -> u.
            dynamics (ca.Function): function of (x, u) -> dx describing the system dynamics.
        """
        self.controller = controller
        self.dynamics = dynamics
        self.ode = {
            'x': self.dynamics.mx_in()[0], 
            # 'u': self.dynamics.mx_in()[1],
            'ode': self.dynamics(self.dynamics.mx_in()[0], 
                                 self.controller(self.dynamics.mx_in()[0]))
        }
        self.integrator = ca.integrator('int', 'rk', self.ode, 0, np.linspace(0, thoriz, int(thoriz/tstep)+1, endpoint=True))

    def __call__(self, x0):
        self.res = self.integrator(x0=x0)['xf']
        return self.res
    def plot(self, fig, ax):
        ax.plot(*np.array(self.res)[:2], color='black', linestyle='dashed')
        s = ax.scatter(*np.array(self.res)[:2], c=np.array(self.res)[2], cmap='coolwarm')
        fig.colorbar(s, ax=ax).set_label('Speed ($[m][s]^{-1}$)')


# load track
default_path = get_test_track()
g = returnPoints()

# fig, ax = plt.subplots(1, 1, sharex=True, sharey=True)
# center_line = g.traj.path.plot_sim(ax)
# print(traj_res[0])

def plot_track():
    #util for displaying track nicely
    def loopify(path):
        return np.concatenate((path[-1:], path), axis=0).T

    left = offset_path(default_path, 2)
    right = offset_path(default_path, -2)

    plt.plot(*loopify(left), color='tab:blue', label='boundary')
    plt.plot(*loopify(right), color='tab:blue')


def simulate_vehicle_movement(X, Y, c, ax):
    for i in range(len(X)):
        ax.scatter(X[i], Y[i], c=c[i]/np.max(c), cmap = "coolwarm")
        plt.pause(0.05)

    return ax.scatter(X, Y, c=c, cmap="coolwarm")



class TrackSim: # relevant sim; variables needed: 
    default_cones = {
        'blue':   offset_path(default_path, 4),
        'yellow': offset_path(default_path, -4),
    }
    def default_cones_func(state, cones, vis_range=10):
        """returns cones within a certain distance. default cone visibility function.

        Args:
            state (np.array): array of shape (5,) defining the current state. First two elements are x, y pos.
            cones (np.array): array of shape (n, 2) defining the positions of all the cones.

        Returns:
            np.array: all visible cones. Shape (2, m).
        """
        dists = np.linalg.norm(cones - state[:2], axis=1)
        return cones[dists<10]
    def __init__(self, 
                 dynamics: ca.Function, 
                 cones: dict[str, np.array] = None,
                 vision_func: callable = None, 
                 tstep=0.1):
        """constructor for simulator with track.

        Args:
            dynamics (ca.Function): Car dynamics. i0 is the state, of shape (5,), i1 is the control input, of shape (2,).
            cones (dict[str, np.array]): dictionary of {'blue': np.array (n, 2), 'yellow': np.array (n, 2)} describing cone locations.
            vision_func (callable): thing to get visible cones from current location.
            tstep (float, optional): control update time. Defaults to 0.1.
        """
        if not cones:
            cones = TrackSim.default_cones
        self.vision_func = vision_func if vision_func else TrackSim.default_cones_func
        self.blue = cones['blue']
        self.yellow = cones['yellow']
        self.dynamics = dynamics
        self.state = DM.zeros(self.dynamics.size_in(0))
        self.ode = {
            'x': self.dynamics.mx_in()[0], 
            'u': self.dynamics.mx_in()[1],
            'ode': self.dynamics(*self.dynamics.mx_in())
        }
        self.intfunc = ca.integrator('int', 'rk', self.ode, 0, tstep)
        self.history = [DM.zeros(self.dynamics.size_in(0))]
    def set_x0(self, x0: DM):
        """clear the history and set the initial state

        Args:
            x0 (DM): the initial state. Shape (5,).
        """
        self.history = [x0]
    def make_step(self, u: DM, x0: None = None):
        """evolve the simulation one step.

        Args:
            u (DM): the control input. Shape (2,).
            x0 (None | DM, optional): the initial position. Defaults to the end position from the last call to make_step.
        """
        if x0:
            self.history = [x0]
        res = self.intfunc(x0=self.history[-1], u=u)['xf']
        self.history.append(res)
    @property
    def x(self):
        """gets current state.

        Returns:
            np.array: current state, in shape (5,)
        """
        return np.array(self.history[-1]).flatten()
    @property
    def hist(self):
        """gets history of simulator.

        Returns:
            np.array: array of history, in shape (n, 5).
        """
        return np.array([np.array(i).flatten() for i in self.history])
    @property
    def visible_cones(self):
        """gets all visible cones at the current location.

        Returns:
            dict[str, np.array]: dictionary of {'blue': np.array (n, 2), 'yellow': np.array (m, 2)} 
                                 describing the set of visible cones of each color.
        """
        blue = self.vision_func(
            np.array(self.history[-1]).flatten(), 
            self.blue
        )
        yellow = self.vision_func(
            np.array(self.history[-1]).flatten(), 
            self.yellow
        )
        return {'blue': blue, 'yellow': yellow}


    def plot(self, fig, ax):
        h = self.hist
        ax.plot(*h[:, :2].T, color='black', linestyle='dashed')
        # s = ax.scatter(*h[:, :2].T, c=h[:, 2], cmap='coolwarm')
        ax.plot(*center_line, linestyle='dashed', color='black', label='centerline')
        plot_track()
        s = simulate_vehicle_movement(*h[:, :2].T, h[:, 2], ax)
        fig.colorbar(s, ax=ax).set_label('Speed ($[m][s]^{-1}$)')


def continuous_dynamics_forces(x, u, l_r=0.5, l_f=0.5, m=1.0):
    """Defines dynamics of the car, i.e. equality constraints.
    parameters:
    state x = [xPos,yPos,v,theta,delta] 
    input u = [F,phi]
    """
    # physical constants
    # l_r  distance rear wheels to center of gravitiy of the car
    # l_f  distance front wheels to center of gravitiy of the car
    # m    mass of the car

    # set parameters
    beta = ca.arctan(l_r/(l_f + l_r) * ca.tan(x[4]))

    # calculate dx/dt
    return ca.vertcat(x[2] * ca.cos(x[3] + beta),  # dxPos/dt = v*cos(theta+beta)
                      x[2] * ca.sin(x[3] + beta),  # dyPos/dt = v*sin(theta+beta)
                      u[0] / m,                    # dv/dt = F/m
                      x[2] / l_r * ca.sin(beta),   # dtheta/dt = v/l_r*sin(beta)
                      u[1])                        # ddelta/dt = phi

def continuous_dynamics(x, u, l_r=0.5, l_f=0.5, m=1.0):
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
    return ca.vertcat(x[2] * ca.cos(x[3] + beta),  # dxPos/dt = v*cos(theta+beta)
                      x[2] * ca.sin(x[3] + beta),  # dyPos/dt = v*sin(theta+beta)
                      u[0] / m,                    # dv/dt = F/m
                      x[2] / l_r * ca.sin(beta))   # dtheta/dt = v/l_r*sin(beta)

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

def discrete_custom_integrator(n = 3, l_r=0.5, l_f=0.5, m=1.0):
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


if __name__ == '__main__':
    # import control as ct
    from kinematic_mpc import KinMPCPathFollower
    from all_settings.all_settings import MPCSettings
    mpc = KinMPCPathFollower(**MPCSettings)
    x, u = MX.sym('x', 5), MX.sym('u', 2)
    
    dynamics = continuous_dynamics(x, u)
    f = ca.Function('f', [x, u], [dynamics])
    c = ca.Function('c', [MX.sym('x', 5)], [DM([0.1, 0.1])])

    A = ca.Function('A', [x], [ca.jacobian(dynamics, x)])
    B = ca.Function('B', [u], [ca.jacobian(dynamics, u)])
    op_pt = np.array([1,0,0.1,1,0.0])
    # K, _, eigs = ct.lqr(A(op_pt), B([0, 0]), np.diag([100,100,10,1,1]), np.diag([1000, 1]))

    # sim = SimpleSim(c, f, 0.1, 10)
    t = TrackSim(f)
    for i in range(100): 

        # t.make_step(-K@(t.x-op_pt))
        sol_dict = mpc.solve()
        t.make_step(sol_dict['u_mpc'][0])
    # sim(x0=DM([0, 0, 0, 0, 0]))

    fig, ax = plt.subplots(1)
    # sim.plot(fig, ax[0])

    t.plot(fig, ax)
    plt.show()

class VehicleSimulationNode(Node):
    def __init__(self, track_sim):
        super().__init__('vehicle_simulation')
        self.track_sim = track_sim
        self.publisher_ = self.create_publisher(Float32MultiArray, 'vehicle_state', 10)
        self.throttle = self.create_subscription(Float64, '/control/throttle', 1)
        self.steer = self.create_subscription(Float64, '/control/steer', 1)
        self.slam = self.create_subscription(Float32MultiArray, 'slam', 1)
        self.control_command = None
    def run(self):
        x, u = MX.sym('x', 5), MX.sym('u', 2)
        
        dynamics = continuous_dynamics(x, u)
        f = ca.Function('f', [x, u], [dynamics])
        c = ca.Function('c', [MX.sym('x', 5)], [DM([0.1, 0.1])])

        A = ca.Function('A', [x], [ca.jacobian(dynamics, x)])
        B = ca.Function('B', [u], [ca.jacobian(dynamics, u)])
        op_pt = np.array([1,0,0.1,1,0.0])
        # K, _, eigs = ct.lqr(A(op_pt), B([0, 0]), np.diag([100,100,10,1,1]), np.diag([1000, 1]))

        # sim = SimpleSim(c, f, 0.1, 10)
        t = TrackSim(f)
        for i in range(100): 
            # t.make_step(-K@(t.x-op_pt))
            t.make_step(self.throttle, self.steer)
        # sim(x0=DM([0, 0, 0, 0, 0]))

        fig, ax = plt.subplots(1)
        # sim.plot(fig, ax[0])

        t.plot(fig, ax)
        plt.show()


if __name__ == '__main__':
    # Initialize ROS 2
    rclpy.init(args=None)

    # Prepare your simulation environment here
    # This includes setting up dynamics, controller, etc.
    # For this example, we'll assume you have a function to set up your TrackSim object
    dynamics = None  # Placeholder for dynamics function setup
    track_sim = TrackSim(dynamics)

    # Create and run the ROS 2 node
    vehicle_simulation_node = VehicleSimulationNode(track_sim)
    rclpy.spin(vehicle_simulation_node)
    print("MPC Controller Down")

    # Cleanup
    vehicle_simulation_node.destroy_node()
    rclpy.shutdown()