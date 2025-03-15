from typing import Dict, List, Tuple
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, MultiLineString, MultiPoint, Polygon
from scipy.spatial import Voronoi, KDTree
from shapely.ops import nearest_points
import networkx as nx
from .TrackMap import graph_from_edges, find_longest_simple_path, edges_in_a_path
from .TrackMap import graph_from_multiline, edges_in_a_cycle, find_longest_cycle

def reinterpolate_points(points, N):
    """ Given some set of points we want to reinterpolate them to get 
    N equidistance points along the line formed by the original points.
    
    Keyword arguments:
    points -- The points which will define a line that we want to interpolate along of
    N -- The number of points we want to interpolate along the line defined by points provided
    """
    coords = [(point.x, point.y) for point in points]
    line = LineString(coords)
    equally_spaced_points = [line.interpolate(distance) for distance in 
                             [i * line.length / (N - 1) for i in range(N)]]
    
    return equally_spaced_points

def are_boundaries_closed(yellow_multiline: MultiLineString, blue_multiline: MultiLineString):
    """ Given two MultiLineStrings which represent track boundaries
    we figure out whether the racetrack will have a closed shape or will not be closed.
    
    Keyword arguments:
    yellow_multiline -- The MultiLineString representing one side of boundaries of the racetrack
    blue_multiline -- The MultiLineString representing the other side of boundaries of the racetrack
    """
    one_is_one = False
    one_more_than_zero = False
    
    for mls in [yellow_multiline, blue_multiline]:
        boundary_graph = graph_from_multiline(mls)
        
        if len(list(nx.connected_components(boundary_graph))) > 1:
            return False

        counter = 0

        cycles = []
        if boundary_graph.is_directed():
            cycles = nx.simple_cycles(boundary_graph)
            print("using directed graph yay")
        else:
            cycles = nx.cycle_basis(boundary_graph)
            print("using undirected hmm??")

        for cycle in cycles:
            counter += 1
            
        if counter == 1 and not one_is_one:
            one_is_one = True
        elif counter > 0:
            one_more_than_zero = True
        
    return one_is_one and one_more_than_zero


def is_point_inside_mls(point, multiline):
    """Given a point and a MultiLineString we check if the point is contained within a region in the MultiLineString.
    
    Keyword arguments:
    point -- The point we want to find whether is contained in the MultiLineString or not
    multiline -- The MultiLineString rerpresenting regions we want to check
    
    """

    boundary_graph = graph_from_multiline(multiline)
    
    for cycle in nx.simple_cycles(boundary_graph):
        cycle_edges = edges_in_a_cycle(cycle)
        mini_coords_list = []
        for cedge in cycle_edges:
            mini_coords_list.extend(cedge)
        polygon = Polygon(mini_coords_list)
        if polygon.contains(point):
            return True
    return False


def N_point_generator(yellow_multiline: MultiLineString, blue_multiline: MultiLineString, N: int, P: int = 400) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
    """This function takes a two MultiLineStrings representing the yellow and blue boundaries of the racetrack and 
    generates N evenly ordered spaced pairs of points along the boundaries of the racetrack.

    Keyword arguments:
    yellow_multiline -- a MultiLineString representing the yellow track boundary
    blue_multiline -- a MultiLineString representing the blue track boundary
    N - the number of pairs to generate
    """
    
    # generate P points on the linestrings
    new_yellow_points = generate_N_points(yellow_multiline, P)
    new_blue_points = generate_N_points(blue_multiline, P)
    
    # create Voronoi diagram 
    all_points = new_yellow_points + new_blue_points
    all_vertices = [(p.x, p.y) for p in all_points]
    vor = Voronoi(all_vertices)

    # find the medial line
    medial_edges = get_medial_line(yellow_multiline, blue_multiline, vor, new_yellow_points, new_blue_points)
    mid_string_points = []
    
    step_size = len(medial_edges)/P
    for i in range(P):
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

    reinterpolated_yellow_points = reinterpolate_points(closest_yellow_points, N)
    reinterpolated_blue_points = reinterpolate_points(closest_blue_points, N)

    normal_reinterpolated_yellow_points = [(point.x, point.y) for point in reinterpolated_yellow_points]
    normal_reinterpolated_blue_points = [(point.x, point.y) for point in reinterpolated_blue_points]
    
    return normal_reinterpolated_yellow_points, normal_reinterpolated_blue_points

def get_medial_line(yellow_multiline: MultiLineString, blue_multiline: MultiLineString, vor: Voronoi, new_yellow_points, new_blue_points) -> List[Tuple[int, int]]: 
    """This function takes a two MultiLineStrings representing the yellow and blue boundaries of the racetrack and 
    gets the medial axis between the two.

    Keyword arguments:
    yellow_multiline -- a MultiLineString representing the yellow track boundary
    blue_multiline -- a MultiLineString representing the blue track boundary
    vor -- the Voronoi diagram of the points from the boundaries
    new_yellow_points
    N - the number of pairs to generate
    """

     # find convex hull to help us filter later on
    all_new_points = new_yellow_points.copy()
    all_new_points.extend(new_blue_points)
    all_convex_hull = MultiPoint(all_new_points).convex_hull
    
    yellow_boundary_graph = graph_from_multiline(yellow_multiline)
    yellow_disjointed = len(list(nx.connected_components(yellow_boundary_graph))) > 1
    
    blue_boundary_graph = graph_from_multiline(blue_multiline)
    blue_disjointed = len(list(nx.connected_components(blue_boundary_graph))) > 1

    # filter the middle voronoi edges
    middle_edges = []
    for edge in vor.ridge_vertices:
        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_string = LineString([vor.vertices[edge[0]], vor.vertices[edge[1]]])
            
            if all_convex_hull.contains(edge_string):
            
                first_point = (vor.vertices[edge[0]][0], vor.vertices[edge[0]][1])
                second_point = (vor.vertices[edge[1]][0], vor.vertices[edge[1]][1])
                
                if yellow_disjointed:
                    if is_point_inside_mls(Point(first_point), yellow_multiline) or is_point_inside_mls(Point(second_point), yellow_multiline):
                        continue
                if blue_disjointed:
                    if is_point_inside_mls(Point(first_point), blue_multiline) or is_point_inside_mls(Point(second_point), blue_multiline):
                        continue

                edge_line = [first_point, second_point]
                if not edge_string.intersects(blue_multiline) and not edge_string.intersects(yellow_multiline):
                    middle_edges.append(edge_line)
                    
    middle_graph = graph_from_edges(middle_edges)
    
    if are_boundaries_closed(yellow_multiline, blue_multiline):
        longest_cycle = find_longest_cycle(middle_graph)
        longest_cycle_edges = edges_in_a_cycle(longest_cycle)
        if longest_cycle_edges != []:
            return longest_cycle_edges
    # otherwise
    longest_path = find_longest_simple_path(middle_graph)
    longest_path_edges = edges_in_a_path(longest_path)
    return longest_path_edges

def generate_N_points(multiline: MultiLineString, N: int) -> List[Tuple[int, int]]:
    """This function takes a MultiLineStrings generates N points along this boundary.

    Keyword arguments:
    multilinee -- a MultiLineString representing a track boundary
    N - the number of points to generate
    """
    total_length = multiline.length
    segment_lengths = np.linspace(0, total_length, N)
    points = [multiline.interpolate(length) for length in segment_lengths]
    return points

def closest_points_on_medial_line(points: List[Tuple[int, int]], medial_line: LineString) -> Dict:
    """This function a set of points and a LineString, and finds the closest set of points on the LineString
        to the points provided.

    Keyword arguments:
    points -- the points we want to pair a closest point to
    medial_line -- a LineString which we want to find points along
    """

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

def find_closest_point(points: List[Tuple[int, int]], location: Tuple[int, int]) -> Tuple[int, int]:
    """This function finds the closest point in a list of points to a location (another point) provided.

    Keyword arguments:
    points -- the list of points we want to search in
    location -- the point we want to find the closest point to
    """
    kdtree = KDTree(points)
    _, closest_point_idx = kdtree.query(location)
    return closest_point_idx, points[closest_point_idx]
