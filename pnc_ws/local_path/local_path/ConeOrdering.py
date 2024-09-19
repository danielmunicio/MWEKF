from all_settings.all_settings import LocalOptSettings
import numpy as np
from feb_msgs.msg import Map
import numpy as np
from .NPointGenerator import N_point_generator
from .TrackMap import find_racetrack, racetrack_to_multiline

def ConeOrdering(msg: Map, state: list[float]):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N)]: pairs of points on the track boundary
    """
    N = LocalOptSettings.N # get size
    
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]).T.tolist(),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]).T.tolist(),
    )

    yellow_edges, blue_edges = find_racetrack(left, right)
    yellow_multiline, blue_multiline = racetrack_to_multiline(yellow_edges, blue_edges)
    leftN_points, rightN_points = N_point_generator(yellow_multiline, blue_multiline, N)
    return leftN_points, rightN_points
