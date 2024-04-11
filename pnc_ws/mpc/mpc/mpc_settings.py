from .Settings import Settings
    
class MPCSettings(metaclass=Settings):
    """settings for CompiledGlobalOpt. All in one place, so it's always synced."""
    N: int = 100
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
