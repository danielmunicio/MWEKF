from all_settings.all_settings import LocalOptSettings
import numpy as np
from feb_msgs.msg import Map
import scipy as sp
from scipy.spatial import Voronoi

def ConeOrdering(msg: Map, state: list[float]):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N)]: pairs of points on the track boundary
    """
    N = LocalOptSettings.N # get size
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]),
    )
    left, right = cone_ordering_algorithm(left, right, N, state)
    return left, right
#%%
def cone_ordering_algorithm(left, right, N, state):
    """even more dummy placeholder algorithm
    """
    
    pts = Voronoi(np.hstack([left, right]).T).vertices
    pos = np.array(state[:2])
    print(np.linalg.norm(np.array(pts)-np.array(pos), axis=1))

    # DOESNT TAKE ANGLE INTO ACCOUNT
    # MIGHT GO BACKWARDS IF SLAM IS SKILL ISSUING
    start = np.argmin(np.linalg.norm(np.array(pts)-np.array(pos), axis=1))

    pts[start], pts[0] = np.copy(pts[0]), np.copy(pts[start])
    print(pts.shape)
    pts = order(pts)
    print(pts)
    pts = interp(pts)(np.linspace(0.0, 1.0, N, endpoint=False))

    return pts, np.copy(pts)


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
    MAX_PATH_LENGTH=15 # meters
    MAX_CONE_SEPARATION_DISTANCE=2.5 # meters
    totaldist = 0.0
    for i in range(len(cones)-2):
        mindex = np.argmin(dists:=np.linalg.norm(cones[i+1:]-cones[i], axis=1))+i+1
        totaldist += dists[mindex-i-1]
        if (dists[mindex-1-i] > MAX_CONE_SEPARATION_DISTANCE) or (totaldist > MAX_PATH_LENGTH):
            return cones[:i+1]
        cones[i+1], cones[mindex] = np.copy(cones[mindex]), np.copy(cones[i+1])
    return cones
cone_ordering_algorithm(np.array([[1,2,3,4,5,6,7,8,9,10], [0,0,0,0,0,0,0,0,0,0]]), np.array([[1,2,3,4,5,6,7,8,9,10], [3,3,3,3,3,3,3,3,3,3]]), 5, [3, 1.5, 0, 0])
# %%
