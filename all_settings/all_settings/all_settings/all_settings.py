from .Settings import Settings
from casadi import Function
import numpy as np
from numpy import pi
from typing import Union

class GlobalOptSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 38
    solver: str = 'ipopt'
    car_params: dict = {'l_r': 0.76, 'l_f':0.76, 'm': 1.}
    bbox: dict = {'l': 2.7+1, 'w': 1.6+1}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -3.0
    ACC_MAX_FN: Union[float, Function] = 2.0
    DF_DOT_MAX: float =  0.5
    V_MIN: float = 0.0
    V_MAX: float = 25.0
    FRIC_MAX = 12.0 # float or function


class LocalOptSettings(metaclass=Settings):
    """settings for LocalOpt. All in one place, so it's always synced."""
    N: int = 10 # for testing, input 10 pairs of cones to test
    solver: str = 'ipopt'
    solver_opts: dict[str:any] = {
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
    car_params: dict = {'l_r': 0.76, 'l_f':0.76, 'm': 1.}
    bbox: dict = {'l': 2.7, 'w': 1.6}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -3.0
    ACC_MAX_FN: Union[float, Function] = 2.0
    DF_DOT_MAX: float =  0.5
    V_MIN: float = 0.0
    V_MAX: float = 10.0
    FRIC_MAX: Union[float, Function] = 12.0


class MPCSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 10
    DT: float = 0.2
    L_F: float = 0.76 - 1
    L_R: float = 0.76 + 1 
    V_MIN: float = 0.0
    V_MAX: float = 25.0
    A_MIN: float = -3.0
    A_MAX: float = 3.0
    DF_MIN: float = -1 * np.pi/10
    DF_MAX: float = 1 * np.pi/10
    A_DOT_MIN: float = -5.0
    A_DOT_MAX: float = 5.0
    DF_DOT_MIN: float = -0.5
    DF_DOT_MAX: float =  0.5
    # Q: list[float] = [1., 1., 10., 0.1]
    Q: list = [5., 5., 0., 0.]
    R: list = [10., 100.]
    # R: list[float] = [0., 0.]
    F: list = [0., 0., 0., 0.]
    TRACK_SLACK_WEIGHT: float = 5e5
    use_rk_dynamics: bool = False
    solver: str = 'ipopt'

class GraphSLAMFastSettings(metaclass=Settings):
    x0: np.ndarray = np.array([0.0, 0.0])
    initial_rows: int = int(1e4)
    initial_cols: int = int(1e4)
    local_radius: int = int(1e5)
    
    max_landmark_distance: float = 1.5
    dx_weight: float = 1.0
    z_weight: float = 5.0

class CANSettings(metaclass=Settings):
    interface: str = 'socketcan'
    channel: str = 'can0'
    bitrate: int = 1000000

class SteeringSettings(metaclass=Settings):
    CAN_SETTINGS: Settings = CANSettings
    MOTOR_TO_WHEELS_RATIO: float = 1.0 # amount motor spins/amount wheels turn
    MAX_MOTOR_SPEED: float = 10.0 # positive, radians/second
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