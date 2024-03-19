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
    order(left)
    order(right)
    l = interp(left)(np.linspace(0.0, 1.0, N, endpoint=False))
    r = interp(right)(np.linspace(0.0, 1.0, N, endpoint=False))

    return l, r

def interp(points):

    dx = np.linalg.norm(np.diff(points, axis=0), axis=1)
    dists = np.cumsum(dx)
    dists = np.hstack([np.array([0.0]), dists])
    
    def interpolate(x):
        x = x*dists[-1]
        i = np.where(x<dists)[0][0]
        t=(x-dists[i-1])/dx[i-1]

        return points[i-1]*(1-t) + points[i]*t
    return lambda x: np.array([interpolate(i) for i in x])



def order(cones):
    """orders the input array by selecting the nearest cone. runs in-place, assumes first cone is correct

    Args:
        cones (ndarray): array of shape (n, 2) with cone points. will be modified in-place. first cone must be correct.
    """
    for i in range(len(cones)-2):
        mindex = np.argmin(np.linalg.norm(cones[i+1:]-cones[i], axis=1))+i+1
        cones[i+1], cones[mindex] = np.copy(cones[mindex]), np.copy(cones[i+1])
