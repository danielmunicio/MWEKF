from all_settings.all_settings import LocalOptSettings
import numpy as np
from feb_msgs.msg import Map
import numpy as np
from .NPointGenerator import N_point_generator
from .TrackMap import find_racetrack, racetrack_to_multiline
from .ConeHistory import ConeHistory
from .GifVisualizer import GifVisualizer
from .Filtering import nearest_neighbor_outlier_removal, filter_cross_boundary_outliers
import time

def ConeOrdering(msg: Map, state: list[float], cone_history: ConeHistory, visualizer: GifVisualizer=None):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N)]: pairs of points on the track boundary
    """
    start_time = time.time()

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

    json_time = time.time()

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
    leftN_points, rightN_points = N_point_generator(yellow_multiline, blue_multiline, bigN, cur_state = state)

    # convert to lists of lists instead of Point objects
    # then check if it needs to be reversed

    leftN_points, rightN_points = list(map(lambda p: [p[0], p[1]], leftN_points)), list(map(lambda p: [p[0], p[1]], rightN_points))

    leftN_points = leftN_points[::int(bigN/N)]
    rightN_points = rightN_points[::int(bigN/N)]

    leftN_points = np.array(leftN_points)
    rightN_points = np.array(rightN_points)
    car_start_position = np.array((0, 0))
    car_start_direction = np.array([1, 0])
    leftN_points, rightN_points = correct_cone_order(leftN_points, rightN_points, car_start_position, direction=car_start_direction)

    algo_time = time.time()

    if visualizer:
        indices_left = list(range(len(leftN_points)))
        indices_right = list(range(len(rightN_points)))
        visualizer.update_gif(left_h, right_h, leftN_points, rightN_points, indices=indices_left + indices_right, state=state)

    gif_time = time.time()

    print(f"JSON writing time: {json_time - start_time} seconds")
    print(f"Algorithm Solve time: {algo_time - json_time} seconds")
    print(f"GIF time: {gif_time - algo_time} seconds")

    return leftN_points, rightN_points

def correct_cone_order(left_cones, right_cones, car_position, direction=np.array([1, 0]), radius_check=None):
    """
    Adjusts the order of the left and right cones based on the car's starting position and its direction.

    Arguments:
        left_cones (np.ndarray): A numpy array of shape (n, 2) representing the ordered (x, y) positions of the left cones.
        right_cones (np.ndarray): A numpy array of shape (n, 2) representing the ordered (x, y) positions of the right cones.
        car_position (np.ndarray): The (x, y) position of the car.
        direction (np.ndarray or float): The direction the vehicle is facing.
            - If a vector: A 2D unit vector representing the car's direction (e.g., np.array([dx, dy])).
            - If an angle: A scalar representing the direction in radians or degrees.

    Returns:
        tuple: A tuple containing the properly ordered left_cones and right_cones (both numpy arrays).
    """

    # Find the closest left and right cones
    left_distances = np.linalg.norm(left_cones - car_position, axis=1)
    right_distances = np.linalg.norm(right_cones - car_position, axis=1)
    
    closest_left_idx = np.argmin(left_distances)
    closest_right_idx = np.argmin(right_distances)
    
    next_index = (closest_left_idx + 1) % len(left_cones)
    prev_index = (closest_left_idx - 1) % len(left_cones)
    
    starting_left_cone = left_cones[closest_left_idx]
    next_left_cone = left_cones[next_index]
    prev_left_cone = left_cones[prev_index]
    
    starting_right_cone = left_cones[closest_left_idx]
    next_right_cone = left_cones[next_index]
    prev_right_cone = left_cones[prev_index]
    
    
    if radius_check is None:
        avg_spacing = np.mean(np.linalg.norm(np.diff(left_cones, axis=0), axis=1))
        radius_check = 2 * avg_spacing
    
    # WITHIN RADIUS CHECK
    next_within_radius = np.linalg.norm(next_left_cone - starting_left_cone) <= radius_check
    prev_within_radius = np.linalg.norm(prev_left_cone - starting_left_cone) <= radius_check
    
    order = np.arange(len(left_cones))
    
    print("closest_left_index = ", closest_left_idx)
    print("next index = ", next_index)
    print("prev index = ", prev_index)
    
    if next_within_radius or prev_within_radius:

        # DOT PRODUCT CHECK
        if next_within_radius and prev_within_radius:
            print("DOT PRODUCT CHECK")
            if isinstance(direction, (int, float)):
                forward_vec = np.array([np.cos(direction), np.sin(direction)])
            else:
                forward_vec = direction / np.linalg.norm(direction)

            vector_next = next_left_cone - starting_left_cone
            vector_prev = prev_left_cone - starting_left_cone
            dot_next = np.dot(vector_next, forward_vec)
            dot_prev = np.dot(vector_prev, forward_vec)
            
            print("dot_next: ", dot_next)
            print("dot_prev: ", dot_prev)
            
            if dot_next > dot_prev:
                print("should be forward order")
            else:
                print("shoudl be in reverse order")
                
            if dot_next > dot_prev:
                start_idx = closest_left_idx
                order = np.arange(len(left_cones))
            else:
                start_idx = closest_left_idx
                order = np.arange(len(left_cones))[::-1]
            
            
        elif next_within_radius:
            print("RADIUS CHECK FORWARD")
            order = np.arange(len(left_cones))  # Forward order
        else:
            print("RADIUS CHECK REVERSE")
            order = np.arange(len(left_cones))[::-1]  # Reverse order
        
    start_pos = np.where(order == closest_left_idx)[0][0]
    left_cones = np.roll(left_cones[order], -start_pos, axis=0)
    right_cones = np.roll(right_cones[order], -start_pos, axis=0)

    return left_cones, right_cones