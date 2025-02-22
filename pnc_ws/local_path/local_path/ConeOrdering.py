from all_settings.all_settings import LocalOptSettings
import numpy as np
from feb_msgs.msg import Map
import numpy as np
from .NPointGenerator import N_point_generator
from .TrackMap import find_racetrack, racetrack_to_multiline
from .ConeHistory import ConeHistory
from .GifVisualizer import GifVisualizer
from .Filtering import nearest_neighbor_outlier_removal, filter_cross_boundary_outliers

def ConeOrdering(msg: Map, state: list[float], cone_history: ConeHistory, visualizer: GifVisualizer=None):
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

    left_h, right_h = None, None
    if LocalOptSettings.use_history:
        cone_history.update_history(left, right, write_to_file=LocalOptSettings.write_to_file)
        left_h, right_h = cone_history.get_history()
        print("length of left history: ", len(left_h))
        print("length of right history: ", len(right_h))
    else:
        left_h, right_h = left, right

    left_history = None
    right_history = None
    # filter points
    if LocalOptSettings.filtering_method == 1:
        test_k = int(0.05*len(left_h))
        test_t = int(0.4*test_k)
        left_history, right_history = nearest_neighbor_outlier_removal(left_h, right_h, k=test_k, threshold=test_t, plot=False)
        print("used nn removal outlier removal")
    elif LocalOptSettings.filtering_method == 2:
        left_history, right_history = filter_cross_boundary_outliers(left_h, right_h, threshold=3.0, plot=False)
        left_history = left_history.tolist()
        right_history = right_history.tolist()
        print("used cross boundary method")
    else:
        left_history, right_history = left_h, right_h
        print("no filtering applied")

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

    if visualizer:
        indices_left = list(range(len(leftN_points)))
        indices_right = list(range(len(rightN_points)))
        visualizer.update_gif(left_h, right_h, leftN_points, rightN_points, indices=indices_left + indices_right)

    return leftN_points, rightN_points


def adjust_cone_order_with_start_position(left_cones, right_cones, car_position, direction=np.array([1, 0])):
    """
    Adjusts the order of the left and right cones based on the car's starting position and its direction.
    
    Keyword Arguments:
        left_cones (np.ndarray): A numpy array of shape (n, 2) representing the ordered (x, y) positions of the left cones.
        right_cones (np.ndarray): A numpy array of shape (n, 2) representing the ordered (x, y) positions of the right cones.
        car_position (np.ndarray): The (x, y) position of the car.
        direction (np.ndarray or float): The direction the vehicle is facing.
            - If a vector: A 2D unit vector representing the car's direction (e.g., np.array([dx, dy])).
            - If an angle: A scalar representing the direction in radians or degrees.
    
    Returns:
        tuple: A tuple containing the potentially reordered left_cones and right_cones (both numpy arrays).
    """
    
    # Find the closest left cone to the car's position
    left_to_car_distance = np.linalg.norm(left_cones - car_position, axis=1)
    closest_left_idx = np.argmin(left_to_car_distance)
    
    # Find the corresponding right cone for the closest left cone
    start_left_cone = left_cones[closest_left_idx]
    start_right_cone = right_cones[closest_left_idx]
    start_cone_type = 'left'
    other_cones = right_cones
    other_cone_type = 'right'
    
    # Alternatively, find the closest right cone to the car's position in case it's closer
    right_to_car_distance = np.linalg.norm(right_cones - car_position, axis=1)
    closest_right_idx = np.argmin(right_to_car_distance)
    
    # If the closest cone to the car's position is the right cone, swap the order
    if right_to_car_distance[closest_right_idx] < left_to_car_distance[closest_left_idx]:
        start_left_cone = left_cones[closest_right_idx]
        start_right_cone = right_cones[closest_right_idx]
        start_cone_type = 'right'
        other_cones = left_cones
        other_cone_type = 'left'
    
    # Step 1: Determine the vehicle's orientation
    if isinstance(direction, np.ndarray):
        forward_vec = direction  # Direction is a vector
    else:
        forward_vec = np.array([np.cos(direction), np.sin(direction)])  # Convert angle to a unit vector
    
    # Step 2: Calculate vectors to the next and previous cones
    next_cone = other_cones[(closest_left_idx + 1) % len(other_cones)]  # Next cone after the starting cone
    prev_cone = other_cones[(closest_left_idx - 1) % len(other_cones)]  # Previous cone before the starting cone
    
    vector_to_next_cone = next_cone - start_left_cone
    vector_to_prev_cone = prev_cone - start_left_cone
    
    plt.quiver(car_position[0], car_position[1], vector_to_next_cone[0], vector_to_next_cone[1], angles='xy', scale_units='xy', scale=1, color='forestgreen')
    plt.quiver(car_position[0], car_position[1], vector_to_prev_cone[0], vector_to_prev_cone[1], angles='xy', scale_units='xy', scale=1, color='forestgreen')
    
    # Normalize the vectors to compare directions
    vector_to_next_cone = vector_to_next_cone / np.linalg.norm(vector_to_next_cone)
    vector_to_prev_cone = vector_to_prev_cone / np.linalg.norm(vector_to_prev_cone)
    
    # Step 3: Compare the vehicle's direction with the directions of the cones
    next_dot_product = np.dot(forward_vec, vector_to_next_cone)
    prev_dot_product = np.dot(forward_vec, vector_to_prev_cone)
    
    # Step 4: If the next cone is more aligned with the vehicle's direction, keep the order
    if next_dot_product > prev_dot_product:
        return left_cones, right_cones
    else:
        # Step 5: Reverse the order of the cones if necessary
        return left_cones[::-1], right_cones[::-1]
