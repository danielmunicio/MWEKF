import numpy as np
from scipy.spatial import distance_matrix, KDTree
import matplotlib.pyplot as plt

def filter_cross_boundary_outliers(left_cones, right_cones, threshold=2.0, plot=False):
    """
    Removes points that are too close to the opposite track boundary.

    Keyword Arguments:
        left_cones (list of lists): Left boundary cone coordinates [[x, y], ...]
        right_cones (list of lists): Right boundary cone coordinates [[x, y], ...]
        threshold (float): Maximum allowed proximity to the opposite boundary before removal.
    """
    left_cones = np.array(left_cones)
    right_cones = np.array(right_cones)

    if len(left_cones) == 0 or len(right_cones) == 0:
        return left_cones, right_cones  # Avoid errors if one set is empty

    # Compute distance matrices
    left_to_right_distances = distance_matrix(left_cones, right_cones)
    right_to_left_distances = distance_matrix(right_cones, left_cones)

    # Find minimum distances to opposite boundary
    min_left_to_right = np.min(left_to_right_distances, axis=1)
    min_right_to_left = np.min(right_to_left_distances, axis=1)

    # Filter out left cones that are too close to the right boundary
    filtered_left = left_cones[min_left_to_right > threshold]

    # Filter out right cones that are too close to the left boundary
    filtered_right = right_cones[min_right_to_left > threshold]
    
    if plot:
        plt.figure(figsize=(8,6))
        plt.scatter(np.array(left_cones)[:,0], np.array(left_cones)[:,1], color='red', label="Original Left", alpha=0.2)
        plt.scatter(np.array(right_cones)[:,0], np.array(right_cones)[:,1], color='blue', label="Original Right", alpha=0.2)
        plt.scatter(filtered_left[:,0], filtered_left[:,1], color='darkred', label="Filtered Left", marker='x')
        plt.scatter(filtered_right[:,0], filtered_right[:,1], color='darkblue', label="Filtered Right", marker='x')
        plt.xlabel("X Axis")
        plt.ylabel("Y Axis")
        plt.legend()
        plt.title("Cross-Boundary Outlier Filtering")
        plt.show()

    return filtered_left, filtered_right


def nearest_neighbor_outlier_removal(left_points, right_points, k=5, threshold=3, plot=False):
    """
    Identifies outliers by counting how many of the k-nearest neighbors belong to the opposite set.
    If more than 'threshold' neighbors are from the opposite set, the point is considered an outlier.
    """
    
    if k == 0 or threshold == 0:
        return left_points, right_points
    
    # Combine all points into a single dataset
    all_points = np.array(left_points + right_points)
    labels = np.array([0] * len(left_points) + [1] * len(right_points))  # 0 = left, 1 = right

    # Build KD-tree from all points
    tree = KDTree(all_points)

    def is_outlier(point, point_label):
        _, neighbor_indices = tree.query(point, k=k+1)  # k+1 because first neighbor is the point itself
        # Get neighbor labels (excluding itself)
        neighbor_labels = labels[neighbor_indices[1:]]  # Skip first index (the point itself)
        # Count how many neighbors belong to the opposite class
        num_opposite_neighbors = np.sum(neighbor_labels != point_label)
        return num_opposite_neighbors > threshold

    # Separate outliers from non-outliers
    left_outliers = [p for p in left_points if is_outlier(p, 0)]
    right_outliers = [p for p in right_points if is_outlier(p, 1)]

    new_left_points = [p for p in left_points if p not in left_outliers]
    new_right_points = [p for p in right_points if p not in right_outliers]

    if plot:
        plt.figure(figsize=(6, 6))
        
        if new_left_points:
            plt.scatter(*zip(*new_left_points), color='yellow', label='Filtered Left')
        if new_right_points:
            plt.scatter(*zip(*new_right_points), color='blue', label='Filtered Right')
        if left_outliers:
            plt.scatter(*zip(*left_outliers), color='magenta', marker='x', label='Left Outliers')
        if right_outliers:
            plt.scatter(*zip(*right_outliers), color='cyan', marker='x', label='Right Outliers')

        plt.legend()
        plt.title("Nearest Neighbor Outlier Removal")
        plt.show()

    return new_left_points, new_right_points