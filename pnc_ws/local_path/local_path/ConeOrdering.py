from all_settings.all_settings import LocalOptSettings
import numpy as np
from feb_msgs.msg import Map
import numpy as np
from .NPointGenerator import N_point_generator
from .TrackMap import find_racetrack, racetrack_to_multiline
from .ConeHistory import ConeHistory

cone_history = ConeHistory()

def ConeOrdering(msg: Map, state: list[float]):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N)]: pairs of points on the track boundary
    """
    N = LocalOptSettings.N # get size
    bigN = 300
    
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]).T.tolist(),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]).T.tolist(),
    )

    cone_history.update_history(left, right)
    left_history, right_history = cone_history.get_history()
    print("length of left history: ", left_history)
    print("length of right history: ", right_history)

    yellow_edges, blue_edges = find_racetrack(left_history, right_history)
    yellow_multiline, blue_multiline = racetrack_to_multiline(yellow_edges, blue_edges)
    leftN_points, rightN_points = N_point_generator(yellow_multiline, blue_multiline, bigN)

    # convert to lists of lists instead of Point objects
    # then check if it needs to be reversed

    leftN_points, rightN_points = list(map(lambda p: [p[0], p[1]], leftN_points)), list(map(lambda p: [p[0], p[1]], rightN_points))
    if (np.linalg.norm((np.array(leftN_points[0])+np.array(rightN_points[0]))/2 - np.array(state[0:2]))
      > np.linalg.norm((np.array(leftN_points[-1])+np.array(rightN_points[-1]))/2 - np.array(state[0:2]))):
        leftN_points, rightN_points = leftN_points[::-1], rightN_points[::-1]

    leftN_points = leftN_points[::int(bigN/N)]
    rightN_points = rightN_points[::int(bigN/N)]

    return leftN_points, rightN_points
