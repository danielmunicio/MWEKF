# from all_settings.all_settings import LocalOptSettings
import numpy as np
from feb_msgs.msg import Map
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

def distance_cone_order(msg: Map, state: list[float]):
    state = np.array(state)
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]).T.tolist(),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]).T.tolist(),
    )
    print(f"Received state {state}")

    left_sorted = []
    pt_cone = state[:2]
    while len(left) > 0:
        left_distances = np.linalg.norm(left - pt_cone, axis=1)
        closest_left_idx = np.argmin(left_distances)
        closest_left = left[closest_left_idx]
        left_sorted.append(closest_left)
        left.pop(closest_left_idx)
        pt_cone = closest_left
    
    right_sorted = []
    pt_cone = state[:2]
    while len(right) > 0:
        right_distances = np.linalg.norm(right - pt_cone, axis=1)
        closest_right_idx = np.argmin(right_distances)
        closest_right = np.array(right[closest_right_idx])
        right_sorted.append(closest_right)
        right.pop(closest_right_idx)
        pt_cone = closest_right
    
    left_interpolated, right_interpolated = interpolate_between_cones(left_sorted, right_sorted)
    return left_interpolated, right_interpolated

def interpolate_between_cones(left_distances, right_distances):
    
    # Left cones
    left_arr = np.array(left_distances)
    left_x = left_arr[:, 0]
    left_y = left_arr[:, 1]
    
    t_left = np.zeros(len(left_x))
    for i in range(1, len(left_x)):
        dx = left_x[i] - left_x[i-1]
        dy = left_y[i] - left_y[i-1]
        t_left[i] = t_left[i-1] + np.sqrt(dx**2 + dy**2)
    
    f_x_left = interp1d(t_left, left_x, kind='cubic')
    f_y_left = interp1d(t_left, left_y, kind='cubic')
    
    # Right cones
    right_arr = np.array(right_distances)
    right_x = right_arr[:, 0]
    right_y = right_arr[:, 1]
    
    t_right = np.zeros(len(right_x))
    for i in range(1, len(right_x)):
        dx = right_x[i] - right_x[i-1]
        dy = right_y[i] - right_y[i-1]
        t_right[i] = t_right[i-1] + np.sqrt(dx**2 + dy**2)
    
    f_x_right = interp1d(t_right, right_x, kind='cubic')
    f_y_right = interp1d(t_right, right_y, kind='cubic')
    

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
