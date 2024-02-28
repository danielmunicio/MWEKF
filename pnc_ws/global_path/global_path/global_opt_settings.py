from Settings import Settings
from casadi import Function
    
class GlobalOptSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 100
    solver: str = 'ipopt'
    car_params: dict[str:float] = {'l_r': 1.4987, 'l_f':1.5213, 'm': 1.}
    bbox: dict[str:float] = {'l': 2, 'w': 1}
    DF_MAX: float  =  0.5
    ACC_MIN: float = -3.0
    ACC_MAX_FN: float|Function = 2.0
    DF_DOT_MAX: float =  0.5
    V_MIN: float = 0.0
    V_MAX: float = 25.0
    FRIC_MAX: float|Function = 12.0

