import numpy as np
from scipy.interpolate import splprep, splev

import settings


def fit_curve_to_points(points_to_fit: np.array):
    # Add one more point so that the spline is a closed loop
    extended_points = np.vstack([points_to_fit, [0, 0]])

    # Generate spline
    tck, u = splprep(extended_points.T, u=None, s=0.0, per=1)
    u_new = np.linspace(u.min(), u.max(), settings.NUM_LINE_POINTS)
    x_new, y_new = splev(u_new, tck, der=0)
    curve = np.vstack([x_new, y_new])
    return curve.T


def compute_track_borders(center_line: np.matrix, track_width: float):
    # Initialize border arrays
    num_points = len(center_line)
    right_border, left_border = np.zeros(
        (num_points - 1, 2)), np.zeros((num_points - 1, 2))

    for i in range(1, num_points):
        # Current centerline coords
        xi, yi = center_line[i, 0], center_line[i, 1]
        # Change in x and y
        dx = xi - center_line[i - 1, 0]
        dy = yi - center_line[i - 1, 1]
        # Distance between consecutive points
        norm = np.sqrt(dx ** 2 + dy ** 2)

        # Go TW/2 far orthogonal to the tangent in each direction
        right_border[i - 1, 0] = xi + track_width / 2 * dy / norm
        right_border[i - 1, 1] = yi - track_width / 2 * dx / norm

        left_border[i - 1, 0] = xi - track_width / 2 * dy / norm
        left_border[i - 1, 1] = yi + track_width / 2 * dx / norm

    return right_border, left_border


def generate_cones(track_border: np.array):
    # Initialize
    cones = []
    num_points = len(track_border)
    i = settings.TRACK_STEP

    # While true - break loop
    while True:
        # Add a cone each settings.TRACK_STEP border point
        current_point = track_border[i, :]
        cones.append(current_point)
        i += settings.TRACK_STEP

        # Terminate when we've gone around one lap
        if i >= num_points:
            break

    return np.array(cones)


def create_track(track_points: np.array):
    center_line = fit_curve_to_points(track_points)
    # Add one more point to close the loop
    center_line = np.vstack([center_line, center_line[1, :]])

    # Compute the track borders
    right_line, left_line = compute_track_borders(
        center_line, settings.TRACK_WIDTH)

    # Generate blue and yellow cones
    yellow_cones = generate_cones(right_line)
    blue_cones = generate_cones(left_line)

    # Generate orange cones settings.TRACK_STEP / 5 border points in each direction from the start
    orange_cones = np.array([
        left_line[settings.TRACK_STEP // 5, :],
        left_line[-settings.TRACK_STEP // 5, :],
        right_line[settings.TRACK_STEP // 5, :],
        right_line[-settings.TRACK_STEP // 5, :],
    ])

    return center_line, right_line, left_line, yellow_cones, blue_cones, orange_cones
