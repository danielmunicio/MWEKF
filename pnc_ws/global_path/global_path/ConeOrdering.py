from global_opt_settings import GlobalOptSettings
import numpy as np
def ConeOrdering(msg):
    """dummy function to represent the cone ordering code.
    Should return two Nx2 arrays of points."""
    N = GlobalOptSettings.N
    return np.zeros((N, 2)), np.ones((N, 2))