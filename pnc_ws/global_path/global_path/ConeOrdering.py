from .global_opt_settings import GlobalOptSettings
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import nearest_points
from .MapPolygon import vertices_in_edges
from .Graph import Graph, find_longest_cycle
from feb_msgs.msg import Map

def ConeOrdering(msg: Map):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N), N]: pairs of points on the track boundary
    """
    # N = GlobalOptSettings.N # get size
    N = 10
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]),
    )
    left, right = N_point_generator(left, right, N)
    return left, right, len(left)

def N_point_generator(yellow_polygon, blue_polygon, N):
    # generate N points on those polygons
    new_yellow_points = generate_N_points(yellow_polygon, N)
    new_blue_points = generate_N_points(blue_polygon, N)
    
    # construct new polygons with N points
    y_polygon = Polygon(new_yellow_points)
    b_polygon = Polygon(new_blue_points)
    
    # create Voronoi diagram
    outer_boundary_vertices = list(y_polygon.exterior.coords)
    inner_boundary_vertices = list(b_polygon.exterior.coords)
    all_vertices = outer_boundary_vertices + inner_boundary_vertices
    vor = Voronoi(all_vertices)
    
    # create medial line
    medial_edges = get_medial_line(y_polygon, b_polygon, vor, N)
    medial_points = list(set(extract_points_from_linestrings(medial_edges)))
    
    # filter medial edges further
    points_in_longest_cycle = filter_medial_line_further(medial_edges, medial_points)
    mid_polygon = Polygon(points_in_longest_cycle)
    
    # in cases where our reconstruction isn't perfect and cannot get a full cycle
    # we have to filter a different way
    if points_in_longest_cycle == []:
        filtered_medial_line = filter_medial_line_by_degree(medial_edges)
        filtered_medial_points = vertices_in_edges(filtered_medial_line)
        mid_polygon = Polygon(filtered_medial_points)
    
    # find closest points on medial line
    cpoml = closest_points_on_medial_line(new_yellow_points, mid_polygon)
    all_points = list(cpoml.keys())
    all_closest_points = [cpoml[point][0] for point in all_points]
    cpoml_2 = closest_points_on_medial_line(all_closest_points, b_polygon)
    
    # find the N yellows and N blue points
    N_yellows = new_yellow_points
    all_points_2 = list(cpoml_2.keys())
    all_closest_points_2 = [cpoml_2[point][0] for point in all_points_2]
    N_blues = all_closest_points_2
    
    # return
    return N_yellows, N_blues

def filter_medial_line_by_degree(medial_edges):
    
    medial_points = list(set(extract_points_from_linestrings(medial_edges)))
    
    medial_dict = {}
    idx = 0

    for point in medial_points:
        medial_dict[point] = idx
        idx += 1

    middle_edges = []
    for lstring in medial_edges:
        middle_edges.extend(linestring_to_edges(lstring))
    
    graph = Graph(len(medial_points))
    for e in middle_edges:
        index_of_e0 = medial_dict[e[0]]
        index_of_e1 = medial_dict[e[1]]
        graph.add_edge(index_of_e0, index_of_e1)
    vertices = [i for i in range(0, len(medial_points))]
    degrees_of_vertices = graph.degrees(vertices)
    
    # figure out stuff to delete
    stuff_to_delete = []
    
    for key in degrees_of_vertices:
        if degrees_of_vertices[key] == 1:
            stuff_to_delete.append(key)
    
    final_edges = []
    
    for e in graph.get_edges():
        if not (e[0] in stuff_to_delete or e[1] in stuff_to_delete):
            first = medial_points[e[0]]
            second = medial_points[e[1]]
            final_edges.append((first, second))
        
    return final_edges
    

def filter_medial_line_further(medial_edges, medial_points):
    medial_dict = {}
    idx = 0
    for point in medial_points:
        medial_dict[point] = idx
        idx += 1
    middle_edges = []
    for lstring in medial_edges:
        middle_edges.extend(linestring_to_edges(lstring))
    
    # construct graph
    graph = Graph(len(medial_points))
    for e in middle_edges:
        index_of_e0 = medial_dict[e[0]]
        index_of_e1 = medial_dict[e[1]]
        graph.add_edge(index_of_e0, index_of_e1)
    
    # find the longest cycle in the graph
    longest_cycle = find_longest_cycle(graph)
    points_in_longest_cycle = []
    for i in longest_cycle:
        points_in_longest_cycle.append(medial_points[i])
    
    return points_in_longest_cycle


def get_medial_line(y_polygon, b_polygon, vor, N): 

    yellow_boundary = LineString(y_polygon.exterior)
    blue_boundary = LineString(b_polygon.exterior)

    # filter edges
    non_boundary_edges = []
    for edge in vor.ridge_vertices:
        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_line = LineString([vor.vertices[edge[0]], vor.vertices[edge[1]]])
            if not edge_line.intersects(blue_boundary) and not edge_line.intersects(yellow_boundary):
                non_boundary_edges.append(edge_line)
    
    # extract medial line
    medial_edges = []
    for edge in vor.ridge_vertices:

        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_line = LineString([vor.vertices[edge[0]], vor.vertices[edge[1]]])
            if not edge_line.intersects(blue_boundary) and not edge_line.intersects(yellow_boundary):
                # check if the edge is within yellow polygon but not blue polygon
                endpoint1 = Point(vor.vertices[edge[0]])
                endpoint2 = Point(vor.vertices[edge[1]])
                if not b_polygon.contains(endpoint1) and not b_polygon.contains(endpoint2) \
                    and y_polygon.contains(endpoint1) and y_polygon.contains(endpoint2):
                    medial_edges.append(edge_line)

    if len(medial_edges) == 0:
        return get_medial_line(y_polygon, b_polygon, N + 50)
    else:
        return medial_edges

def extract_points_from_linestrings(linestrings):
    points = []
    for linestring in linestrings:
        coords = list(linestring.coords)
        points.extend(coords)
    return points

def linestring_to_edges(linestring):
    edges = []
    coords = list(linestring.coords)
    num_coords = len(coords)
    for i in range(num_coords - 1):
        edges.append((coords[i], coords[i+1]))
    if linestring.is_closed:
        edges.append((coords[-1], coords[0]))
    return edges

def extract_points_from_linestrings(linestrings):
    points = []
    for linestring in linestrings:
        coords = list(linestring.coords)
        points.extend(coords)
    return points

def generate_N_points(polygon, N):
    boundary = polygon.boundary
    points = [boundary.interpolate(i / N, normalized=True) for i in range(N)]
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
