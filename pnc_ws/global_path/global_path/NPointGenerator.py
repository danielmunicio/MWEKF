import math
import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import Delaunay
from itertools import permutations
from shapely.geometry import Polygon, Point, LineString, MultiLineString, MultiPoint
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.ops import nearest_points, linemerge
from TrackMap import graph_from_edges, find_longest_simple_path

def N_point_generator(yellow_multiline, blue_multiline, N):
    
    # generate N points on the linestrings
    new_yellow_points = generate_N_points(yellow_multiline, N)
    new_blue_points = generate_N_points(blue_multiline, N)
    
    # create Voronoi diagram 
    all_points = new_yellow_points + new_blue_points
    all_vertices = [(p.x, p.y) for p in all_points]
    vor = Voronoi(all_vertices)

    # find the medial line
    medial_edges = get_medial_line(yellow_multiline, blue_multiline, vor, new_yellow_points, new_blue_points, N)
    mid_string_points = []
    
    step_size = len(medial_edges)/N
    for i in range(N):
        index = int(round(step_size*i))
        if index < len(medial_edges):
            me = medial_edges[int(round(step_size*i))]
            mid_string_points.append(Point(me[0]))
        else:
            break
        
    # find the closest yellow points to those green points
    cpoml_y = closest_points_on_medial_line(mid_string_points, yellow_multiline)
    y_points = list(cpoml_y.keys())
    closest_yellow_points = [cpoml_y[point][0] for point in y_points]

    # find the closest blue points to those green points
    cpoml_b = closest_points_on_medial_line(mid_string_points, blue_multiline)
    b_points = list(cpoml_b.keys())
    closest_blue_points = [cpoml_b[point][0] for point in b_points]
    
    return closest_yellow_points, closest_blue_points

def get_medial_line(yellow_multiline, blue_multiline, vor, new_yellow_points, new_blue_points, N): 

    # find convex hulls to help us filter later on
    yellow_convex_hull = MultiPoint(new_yellow_points).convex_hull
    blue_convex_hull = MultiPoint(new_blue_points).convex_hull

    # filter the middle voronoi edges
    middle_edges = []
    for edge in vor.ridge_vertices:
        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_string = LineString([vor.vertices[edge[0]], vor.vertices[edge[1]]])
            first_point = (vor.vertices[edge[0]][0], vor.vertices[edge[0]][1])
            second_point = (vor.vertices[edge[1]][0], vor.vertices[edge[1]][1])
            edge_line = [first_point, second_point]
            if not edge_string.intersects(blue_multiline) and not edge_string.intersects(yellow_multiline):
                middle_edges.append(edge_line)
    
    # remove extraneous edges from the medial line by finding longest path in the graph
    middle_graph = graph_from_edges(middle_edges)
    longest_path = find_longest_simple_path(middle_graph)
    
    longest_path_edges = []
    for i in range(len(longest_path) - 1):
        current_point = Point(longest_path[i])
        if yellow_convex_hull.contains(current_point) or blue_convex_hull.contains(current_point):
            string = [longest_path[i], longest_path[i+1]]
            longest_path_edges.append(string)
            
    # if the figure forms a polygon, add a final cycl edge
    cycle_edge = LineString([longest_path[-1], longest_path[0]])
    if not (cycle_edge.intersects(yellow_multiline) or cycle_edge.intersects(blue_multiline)):
        longest_path_edges.append([longest_path[-1], longest_path[0]])
    
    return longest_path_edges

def generate_N_points(multiline, N):
    total_length = multiline.length
    segment_lengths = np.linspace(0, total_length, N)
    points = [multiline.interpolate(length) for length in segment_lengths]
    return points

def closest_points_on_medial_line(points, medial_line):

    closest_points = {}
    # Iterate over each point on the outer boundary
    for p in points:
        # Project the outer boundary point onto the medial line
        projected_point, closest_point_on_medial = nearest_points(p, medial_line)
        # Calculate the distance between the outer boundary point and its projection
        distance = p.distance(projected_point)
        # Store the closest point and its distance for this outer boundary point
        closest_points[p] = (closest_point_on_medial, distance)
    
    return closest_points

def find_closest_point(points, location):
    kdtree = KDTree(points)
    dist, closest_point_idx = kdtree.query(location)
    return closest_point_idx, points[closest_point_idx]

def should_reverse(points, car_location, heading_vector):
    
    # figure out the closest point
    i, closest_point = find_closest_point(points, car_location)
    next_i = i+1
    previous_i = i-1
    if next_i >= len(points):
        next_i = 0
    if previous_i < 0:
        previous_i = len(points) - 1
    
    # check the angles between the next and previous points
    
    prev_point = points[previous_i]
    next_point = points[next_i]
    prev_vector = (prev_point[0] - closest_point[0], prev_point[1] - closest_point[1])
    next_vector = (next_point[0] - closest_point[0], next_point[1] - closest_point[1])

    dot_prev = dot_product(heading_vector, prev_vector)
    dot_next = dot_product(heading_vector, next_vector)

    if dot_prev > 0:
        return True
    else:
        return False
