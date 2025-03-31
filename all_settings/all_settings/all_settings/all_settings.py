from .Settings import Settings
from casadi import Function
import numpy as np
from numpy import pi
from typing import Union, Dict

class GlobalOptSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 100
    solver: str = 'ipopt'
    car_params: dict = {'l_r': 1.58/2, 'l_f':1.58/2, 'm': 1.}
    # car_params: dict = {'l_r': 1.53/2, 'l_f': 1.53/2, 'm': 1.}
    bbox: dict = {'l': 2.7+0.5, 'w': 1.6+0.5}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -4.5
    ACC_MAX_FN: Union[float, Function] = 2.5
    DF_DOT_MAX: float =  1.0
    V_MIN: float = 0.0
    V_MAX: float = 8.0
    FRIC_MAX = 7.0 # float or function
    write_to_file = True
    N_CONES_PER_POINT: int = 4


class LocalOptSettings(metaclass=Settings):
    """settings for LocalOpt. All in one place, so it's always synced."""
    N: int = 50 # for testing, input 10 pairs of cones to test
    solver: str = 'ipopt'
    solver_opts: Dict = {
        # set linear solver
        'ipopt.linear_solver': 'ma57',
        # expand MX expressions to SX
        'expand': True,
        # make ipopt shut up
        'ipopt.print_level': 0,
        'ipopt.sb': 'yes',
        'ipopt.ma57_automatic_scaling': 'no',
        'print_time': 0,
    }
    # car_params: dict[str:float] = {'l_r': 1.4987, 'l_f':1.5213, 'm': 1.}1.201
    # car_params: dict = {'l_r': 0.79+0.6, 'l_f':0.79-0.6, 'm': 1.}
    car_params: dict = {'l_r': 1.58/2, 'l_f': 1.58/2, 'm': 1.}
    bbox: dict = {'l': 2.7, 'w': 1.6}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -4.5
    ACC_MAX_FN: Union[float, Function] = 2.5
    DF_DOT_MAX: float =  1.0
    V_MIN: float = 0.0
    V_MAX: float = 10.0
    FRIC_MAX: Union[float, Function] = 7.0
    write_to_file = False
    save_to_gif = False
    use_history = True
    filtering_method = 0
    distance_cone_order: bool = False

class FATROPSolver(metaclass=Settings):
    name = 'fatrop'
    opts = {'structure_detection': 'auto', 'expand': False, 'debug': False, 'fatrop.print_level': -1, 'print_time': False} # 'equality' key must be passed by user!
class IPOPTSolver(metaclass=Settings):
    name = 'ipopt'
    opts = {'expand': False, 'ipopt.linear_solver': 'MA27', 'print_time': False, 'ipopt.print_level': 0} # 'equality' key must be passed by user!

class RK4Solver(metaclass=Settings):
    n=1
    method='rk4'

class MidpointSolver(metaclass=Settings):
    n=3
    method='midpoint'

class MPCSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 10
    DT: float = 0.1
    L_F: float = 1.58/2#0.79+0.6
    L_R: float = 1.58/2#
    V_MIN: float = 0.0
    V_MAX: float = 10.0
    A_MIN: float = -5.0
    A_MAX: float = 3.0
    DF_MIN: float = -0.5
    DF_MAX: float = 0.5
    A_DOT_MIN: float = -10.0
    A_DOT_MAX: float = 10.0
    DF_DOT_MIN: float = -1.0
    DF_DOT_MAX: float = 1.0
    Q: list = [5., 5., 0., 0., 0.]
    R: list = [0., 0.]
    RUNTIME_FREQUENCY: float = 100
    nlpsolver = FATROPSolver
    ivpsolver = MidpointSolver
    PUBLISH: bool = True



class GraphSLAMSolverSettings(metaclass=Settings):
    x0: np.ndarray = np.array([0.0, 0.0])
    initial_rows: int = int(1e4)
    initial_cols: int = int(1e4)
    local_radius: int = int(1e5)
    max_landmark_distance: float = 1.0
    dx_weight: float = 1.0
    z_weight: float = 5.0

class GraphSLAMSettings(metaclass=Settings):
    publish_to_rviz: bool = True
    local_radius: int = int(10)
    local_vision_delta: float = pi / 2 # how far into periphery of robot heading on each side to include local cones (robot has tunnel vision if this is small) (radians)
    solve_by_time: bool = True # Solve based on distance traveled otherwise
    solve_frequency: float = 0.3 # Time in seconds between solves
    solve_distance: float = 1.0 # Distance in meters between solves 
    using_wheelspeeds: bool = True

    # Simulator Based Settings
    using_simulator: bool = True
    # Whether or not to use ground truth (perfectly accurate) measurements or not 
    using_ground_truth_cones: bool = False
    using_ground_truth_wheelspeeds: bool = False # Whether or not to use the perfect wheel speeds
    bypass_SLAM = False # Bypasses SLAM Entirely, publishes ground truth position
    instant_global_map = False # Whether or not to instantly publish the global map
    # Hardware Based Settings
    forward_imu_direction: str = 'x' # Which direction is forward for the IMU. Can  be 'x', 'y', or 'z'
    NUM_LAPS: int = 3

class GraphSLAMRSSolverSettings(metaclass=Settings):
    x0: tuple[float, float] = (0.0, 0.0)
    max_landmark_distance: float = 0.5
    dx_weight: float = 1.0
    z_weight: float = 1.0
    dclip: dict = {1: 0.5, 2: 0.5, 3: 10.0} # keys are color values
    max_icp_steps: int = 0 # only used when data_association_strategy=1
    max_newton_steps: int = 0
    data_association_strategy: int = 0 # can be 1 (true ICP) or 0 (global optimization thing)


class MPC2Settings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 10
    DT: float = 0.1
    L_F: float = 0.76
    L_R: float = 0.76
    V_MIN: float = 0.0
    V_MAX: float = 15.0
    A_MIN: float = -3.0
    A_MAX: float = 2.0
    DF_MIN: float = -0.6
    DF_MAX: float = 0.6
    A_DOT_MIN: float = -5.0
    A_DOT_MAX: float = 5.0
    DF_DOT_MIN: float = -0.5
    DF_DOT_MAX: float =  0.5
    # Q: list[float] = [1., 1., 10., 0.1]
    Q: list = [5., 5., 0., 0., 0.]
    R: list = [10., 100.]
    # R: list[float] = [0., 0.]
    # F: list = [0., 0., 0., 0.]
    TRACK_SLACK_WEIGHT: float = 5e5
    use_rk_dynamics: bool = False
    solver: str = 'ipopt'
    PUBLISH: bool = False
class CANSettings(metaclass=Settings):
    interface: str = 'socketcan'
    channel: str = 'can0'
    bitrate: int = 1000000

class SteeringSettings(metaclass=Settings):
    CAN_SETTINGS: Settings = CANSettings
    MOTOR_TO_WHEELS_RATIO: float = 1.0 # amount motor spins/amount wheels turn
    MAX_MOTOR_POS: float = 1.35 # positive radians, furthest distance from zero (max travel/2)
    MAX_MOTOR_SPEED: float = 50.0 # positive, radians/second
    MAX_MOTOR_ACC: float = 1000.0 # positive, radians/second/second
    MOTOR_TICKS_PER_RAD: float = 18000/pi # ticks of motor encoder per radian of motor turn

class BBWSerialSettings(metaclass=Settings):
    port: str = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_75033303634351C041A1-if00'
    baudrate: int = 115200
    timeout: float = 0.05

class BrakingSettings(metaclass=Settings):
    SERIAL_SETTINGS: Settings = BBWSerialSettings
    VOLTS_PER_PSI: float = 0.0271494 # computed using linear regression on desmos with like 3 datapoints
    VOLTS_FOR_ZERO_PSI: float = 0.962366 # same regression ^
    VMAX: float = 4.7 # max voltage the arduino can output (at PWM duty cycle 255)
