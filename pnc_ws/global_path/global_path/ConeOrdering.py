from .global_opt_settings import GlobalOptSettings
import numpy as np
from feb_msgs.msg import Map
def ConeOrdering(msg: Map):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N)]: pairs of points on the track boundary
    """
    N = GlobalOptSettings.N # get size
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]),
    )
    left, right = cone_ordering_algorithm(left, right, N)
    return left, right

def cone_ordering_algorithm(left, right, N):
    """even more dummy placeholder algorithm
    """
    return np.ones((N, 2)), np.ones((N, 2))