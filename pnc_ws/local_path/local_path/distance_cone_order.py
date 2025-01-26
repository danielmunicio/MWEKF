# from all_settings.all_settings import LocalOptSettings
import numpy as np
from feb_msgs.msg import Map
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

def distance_cone_order(msg: Map, state: list[float]):
    state = np.array(state)
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]).T,
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]).T,
    )
    print(f"Received state {state}")
    print("LEFT: ", left)
    print("RIGHT: ", right)
    left_sorted = []
    pt_cone = state[:2]

    # Filter out cones behind the car
    heading = state[2] 
    rotation = lambda theta: np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    rotation_matrix = rotation(-heading)

    filtered_left = np.zeros((0, 2))
    filtered_right = np.zeros((0, 2))

    for cone in left:
        cone_rel = cone - pt_cone
        cone_in_car_frame = rotation_matrix @ cone_rel
        if cone_in_car_frame[0] > 0:
            filtered_left = np.vstack([filtered_left, [cone[0], cone[1]]])
        
    for cone in right:
        cone_rel = cone - pt_cone
        cone_in_car_frame = rotation_matrix @ cone_rel
        if cone_in_car_frame[0] > 0:
            filtered_right = np.vstack([filtered_right, [cone[0], cone[1]]])

    pt_cone = np.array(pt_cone)
    print(pt_cone)
    while len(filtered_left) > 0:
        left_distances = np.linalg.norm(filtered_left - pt_cone, axis=1)
        closest_left_idx = np.argmin(left_distances)
        closest_left = filtered_left[closest_left_idx]
        left_sorted.append(closest_left)
        filtered_left = np.delete(filtered_left, closest_left_idx, axis=0)
        print("NEW CLOSETST LEFT: ", closest_left)
        pt_cone = closest_left
    
    right_sorted = []
    pt_cone = state[:2]
    print("Made it here")
    while len(filtered_right) > 0:
        right_distances = np.linalg.norm(filtered_right - pt_cone, axis=1)
        closest_right_idx = np.argmin(right_distances)
        closest_right = np.array(filtered_right[closest_right_idx])
        right_sorted.append(closest_right)
        filtered_right = np.delete(filtered_right, closest_right_idx, axis=0)
        pt_cone = closest_right
    
    if (len(right_sorted) < 1 or len(left_sorted) < 1):
        raise RuntimeError
    left_interpolated, right_interpolated = interpolate_between_cones(left_sorted, right_sorted)
    return left_interpolated, right_interpolated

def interpolate_between_cones(left_distances, right_distances):
    print("LEFT: ", left_distances)
    print("RIGHT: ", right_distances)
    # Left cones
    left_arr = np.array(left_distances)
    print("LEFTARR: ", left_arr)
    left_x = left_arr[:, 0]
    left_y = left_arr[:, 1]
    
    t_left = np.zeros(len(left_x))
    for i in range(1, len(left_x)):
        dx = left_x[i] - left_x[i-1]
        dy = left_y[i] - left_y[i-1]
        t_left[i] = t_left[i-1] + np.sqrt(dx**2 + dy**2)
    
    f_x_left = interp1d(t_left, left_x, kind='linear')
    f_y_left = interp1d(t_left, left_y, kind='linear')
    
    # Right cones
    right_arr = np.array(right_distances)
    right_x = right_arr[:, 0]
    right_y = right_arr[:, 1]
    
    t_right = np.zeros(len(right_x))
    for i in range(1, len(right_x)):
        dx = right_x[i] - right_x[i-1]
        dy = right_y[i] - right_y[i-1]
        t_right[i] = t_right[i-1] + np.sqrt(dx**2 + dy**2)
    
    f_x_right = interp1d(t_right, right_x, kind='linear')
    f_y_right = interp1d(t_right, right_y, kind='linear')
    

    num_points = 10
    t_new_left = np.linspace(0, t_left[-1], num_points)
    t_new_right = np.linspace(0, t_right[-1], num_points)
    
    left_interpolated = np.column_stack((f_x_left(t_new_left), f_y_left(t_new_left)))
    right_interpolated = np.column_stack((f_x_right(t_new_right), f_y_right(t_new_right)))
    
    return left_interpolated, right_interpolated

def test_interpolation():
    left_distances = np.array([[0, 0], [1, 0], [2, 1], [2, 2]])
    right_distances = np.array([[0, 2], [2, 2], [3, 3], [3, 4]])
    left_interpolated, right_interpolated = interpolate_between_cones(left_distances, right_distances)
    print(left_interpolated)
    print(right_interpolated)

    return left_interpolated, right_interpolated

def plot_interpolation(left, right):
    plt.plot(left[:, 0], left[:, 1], 'ro-', label='Left')
    plt.plot(right[:, 0], right[:, 1], 'bo-', label='Right')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    left, right = test_interpolation()
    plot_interpolation(left, right)
