from .Settings import Settings
from casadi import Function

class GlobalOptSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 100
    solver: str = 'ipopt'
    car_params: dict[str:float] = {'l_r': 0.76, 'l_f':0.76, 'm': 1.}
    bbox: dict[str:float] = {'l': 2.7, 'w': 1.6}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -3.0
    ACC_MAX_FN: float|Function = 2.0
    DF_DOT_MAX: float =  0.5
    V_MIN: float = 0.0
    V_MAX: float = 25.0
    FRIC_MAX: float|Function = 12.0


class LocalOptSettings(metaclass=Settings):
    """settings for LocalOpt. All in one place, so it's always synced."""
    N: int = 10 # for testing, input 10 pairs of cones to test
    solver: str = 'ipopt'
    # car_params: dict[str:float] = {'l_r': 1.4987, 'l_f':1.5213, 'm': 1.}1.201
    car_params: dict[str:float] = {'l_r': 0.76, 'l_f':0.76, 'm': 1.}
    bbox: dict[str:float] = {'l': 2.7, 'w': 1.6}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -3.0
    ACC_MAX_FN: float|Function = 2.0
    DF_DOT_MAX: float =  0.5
    V_MIN: float = 0.0
    V_MAX: float = 25.0
    FRIC_MAX: float|Function = 12.0


class MPCSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 10
    DT: float = 0.2
    L_F: float = 1.5213
    L_R: float = 1.4987
    V_MIN: float = 0.0
    V_MAX: float = 25.0
    A_MIN: float = -3.0
    A_MAX: float = 2.0
    DF_MIN: float = -0.5
    DF_MAX: float = 0.5
    A_DOT_MIN: float = -1.5
    A_DOT_MAX: float = 1.5
    DF_DOT_MIN: float = -0.5
    DF_DOT_MAX: float =  0.5
    Q: list[float] = [1., 1., 10., 0.1]
    R: list[float] = [10., 100.]     
    F: list[float] = [0., 0., 0., 10.]
    TRACK_SLACK_WEIGHT: float = 5e5
    use_rk_dynamics: bool = False
    solver: str = 'ipopt'


