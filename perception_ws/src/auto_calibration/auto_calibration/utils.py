import numpy as np
import random



def plane_RANSAC(points: np.array, num_iterations, tol):
    num_points = points.shape[0]
    best_inliers = []
    best_num_inliers = 0
    
    
    for _ in range(num_iterations):
        sample = random.sample(range(num_points), 3)
        p1, p2, p3 = points[sample[0], :], points[sample[1], :], points[sample[2], :]
        
        v1 = p2 - p1
        v2 = p3 - p1
        
        plane_normal = np.cross(v1, v2)
        plane_normal /= np.linalg.norm(plane_normal)
        distance = np.dot(-plane_normal, p1.T)
        
        inliers = []
        for i, point in enumerate(points):
            dist = abs(np.dot(plane_normal, point) + distance)
            if dist < tol:
                inliers.append(i)

        num_inliers = len(inliers)

        if num_inliers > best_num_inliers:
            best_inliers = inliers
            best_num_inliers = num_inliers
        
    return points[best_inliers, :]

        



def filter_points(current_pc: np.array, cp_matrix: np.array, tol = 0.05):
    v1 = cp_matrix[:, 1] - cp_matrix[:, 0]
    v2 = cp_matrix[:, 2] - cp_matrix[:, 0]
    
    plane_normal = np.matrix(np.cross(v1, v2))
    plane_normal /= np.linalg.norm(plane_normal)
    d = np.dot(-plane_normal, cp_matrix[:, 0])[0, 0]
    filtered_points = cp_matrix
    
    for p in current_pc:
        vec = np.matrix([p[0], p[1], p[2]]).T
        distance = np.abs(np.dot(plane_normal, vec)[0, 0] + d)
                    
        if distance < tol:
            filtered_points = np.hstack((filtered_points, vec))
    
    return filtered_points


def check_core_point(eps, minPts, points, index):
    # get points from given index
    current_point = points[:, index]

    # check available points within radius
    distances = np.linalg.norm(points - current_point, axis=0)
    neighbour_indices = np.where((distances <= eps) & (distances > 0))[0]
    num_neighbours = len(neighbour_indices)

    # check how many points are present within radius
    if num_neighbours >= minPts:
        # return format (dataframe, is_core, is_border, is_noise)
        return (neighbour_indices, True, False, False)

    elif num_neighbours < minPts and num_neighbours > 0:
        # return format (dataframe, is_core, is_border, is_noise)
        return (neighbour_indices, False, True, False)

    elif num_neighbours == 0:
        # return format (dataframe, is_core, is_border, is_noise)
        return (neighbour_indices, False, False, True)


def cluster_with_stack(eps, minPts, points):
    # initiating cluster number
    cluster_counter = 1
    # initiating stacks to maintain
    current_stack = set()
    unvisited = list(range(points.shape[1]))
    clusters = []

    while (len(unvisited) != 0):  # run until all points have been visited
        # identifier for first point of a cluster
        first_point = True
        # choose a random unvisited point
        current_stack.add(random.choice(unvisited))

        while len(current_stack) != 0:  # run until a cluster is complete
            # pop current point from stack
            curr_idx = current_stack.pop()

            # check if point is core, neighbour or border
            neigh_indexes, iscore, isborder, isnoise = check_core_point(
                eps, minPts, points, curr_idx)

            # dealing with an edge case
            if (isborder & first_point):
                # for first border point, we label it and its neighbours as noise
                clusters.append((curr_idx, 0))
                clusters.extend(
                    list(zip(neigh_indexes, [0 for _ in range(len(neigh_indexes))])))

                # label as visited
                unvisited.remove(curr_idx)
                unvisited = [e for e in unvisited if e not in neigh_indexes]
                continue
            unvisited.remove(curr_idx)  # remove point from unvisited list
            neigh_indexes = set(neigh_indexes) & set(
                unvisited)  # look at only unvisited points

            if iscore:  # if current point is a core
                first_point = False

                # assign to a cluster
                clusters.append((curr_idx, cluster_counter))
                # add neighbours to a stack
                current_stack.update(neigh_indexes)

            elif isborder:  # if current point is a border point
                clusters.append((curr_idx, cluster_counter))
                continue

            elif isnoise:  # if current point is noise
                clusters.append((curr_idx, 0))
                continue

        if not first_point:
            # increment cluster number
            cluster_counter += 1

    return clusters
