from .Settings import Settings
from casadi import Function
import numpy as np
from numpy import pi
from typing import Union, Dict

class GlobalOptSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 50
    solver: str = 'ipopt'
    car_params: Dict = {'l_r': 0.76, 'l_f':0.76, 'm': 1.}
    bbox: Dict = {'l': 2.7+1, 'w': 1.6+1}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -3.0
    ACC_MAX_FN: Union[float, Function] = 2.0
    DF_DOT_MAX: float =  0.5
    V_MIN: float = 0.0
    V_MAX: float = 25.0
    FRIC_MAX = 12.0 # float or function
    write_to_file = True


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
    car_params: dict = {'l_r': 0.76, 'l_f':0.76, 'm': 1.0}
    bbox: dict = {'l': 2.7, 'w': 1.6}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -3.0
    ACC_MAX_FN: Union[float, Function] = 2.0
    DF_DOT_MAX: float =  0.5
    V_MIN: float = 0.0
    V_MAX: float = 15.0
    FRIC_MAX: Union[float, Function] = 12.0
    write_to_file = True
    save_to_gif = True
    use_history = True
    filtering_method = 0
    distance_cone_order: bool = False

class MPCSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 50
    DT: float = 0.2
    L_F: float = 0.76 - 1
    L_R: float = 0.76 + 1 
    V_MIN: float = 0.0
    V_MAX: float = 15.0
    A_MIN: float = -3.0
    A_MAX: float = 3.0
    DF_MIN: float = -1 * np.pi/10
    DF_MAX: float = 1 * np.pi/10
    A_DOT_MIN: float = -5.0
    A_DOT_MAX: float = 5.0
    DF_DOT_MIN: float = -0.6
    DF_DOT_MAX: float =  0.5
    # Q: list[float] = [1., 1., 10., 0.1]
    Q: list = [5., 5., 0., 0.]
    R: list = [10., 100.]
    # R: list[float] = [0., 0.]
    F: list = [0., 0., 0., 0.]
    TRACK_SLACK_WEIGHT: float = 5e5
    use_rk_dynamics: bool = False
    solver: str = 'ipopt'

class GraphSLAMSolverSettings(metaclass=Settings):
    x0: np.ndarray = np.array([0.0, 0.0])
    initial_rows: int = int(1e4)
    initial_cols: int = int(1e4)
    local_radius: int = int(10)
    
    max_landmark_distance: float = 0.7
    dx_weight: float = 1.0
    z_weight: float = 1.0

class GraphSLAMSettings(metaclass=Settings):
    publish_to_rviz: bool = True
    local_radius: int = int(1e5)
    local_vision_delta: float = pi / 2 # how far into periphery of robot heading on each side to include local cones (robot has tunnel vision if this is small) (radians)
    solve_by_time: bool = True # Solve based on distance traveled otherwise
    solve_frequency: float = 0.3 # Time in seconds between solves
    solve_distance: float = 1.0 # Distance in meters between solves 
    using_wheelspeeds: bool = True

    # Simulator Based Settings
    using_simulator: bool = True
    # Whether or not to use ground truth (perfectly accurate) measurements or not 
    using_ground_truth_cones: bool = True
    using_ground_truth_wheelspeeds: bool = True # Whether or not to use the perfect wheel speeds
    bypass_SLAM: bool= False # Bypasses SLAM Entirely, publishes ground truth position
    instant_global_map: bool = False # Whether or not to instantly publish the global map
    # Hardware Based Settings
    forward_imu_direction: str = 'x' # Which direction is forward for the IMU. Can  be 'x', 'y', or 'z'

    # Whether or not we are using MWEKF Implementation
    using_mwekf: bool = True

class MWEKFSettings(metaclass=Settings):
    window_size: int = 10 # num of cones in window

class SimulatorPerceptionSettings(metaclass=Settings):
    camera_noise_std: float = 0.35
    lidar_noise_std: float = 0.2

    # Delay for Camera and LiDAR BETWEEN measurement and send
    camera_delay_mean: float = 0.01
    camera_delay_std: float = 1e-5

    lidar_delay_mean: float = 0.01
    lidar_delay_std: float = 1e-5

    # Publishing Rate for Sensor 
    camera_hz: int = 15
    lidar_hz: int = 5

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


class LiDAROnlySettings(metaclass=Settings):
    # Ground Filtering 
    #ground_plane_coefficients: list = [-0.055, -0.003, 0.998, 0.591]
    ground_plane_coefficients: list = [-0.25, 0.002, 1.0, 0.680]
    ground_filter_threshold: float = 0.05
    ground_filter_top_threshold: float = 0.5
    max_distance: float = 10.0

    # DB Scan 
    eps: float = 0.1
    min_samples: float = 30
    using_cuML: bool = True

    # Cone Dimensions Requirements
    # List of [min_size, max_size]
    x_size: list = [0.1, 0.25]
    y_size: list = [0.05, 0.15]
    z_size: list = [0.1, 0.3]

    # Cone Position Algorithm 
    # Can do 'median' or 'mean
    method: str = 'median' 

    # Visual Settings: 
    publish_ground_filtered_pointcloud: bool = True
    publish_perception_viz: bool = True

class CameraOnlySettings(metaclass=Settings):
    publish_visual: bool = False
    yolo_minimum_confidence: float = 0.7

    # Camera setup settings
    dual_camera: bool = False
    logitech_camera: bool = False
    realsense_camera: bool = True