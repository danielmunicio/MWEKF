from .local_opt_settings import LocalOptSettings
from feb_msgs.msg import Map
import numpy as np
import scipy as sp
from scipy.spatial import Voronoi
import math
from scipy.spatial import KDTree
from shapely.geometry import Polygon, Point, LineString, MultiLineString
from shapely.ops import linemerge
from .MapPolygon import find_polygons_from_points
from .Graph import Graph

def ConeOrdering(msg: Map, state: list[float]):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N)]: pairs of points on the track boundary
    """
    N = LocalOptSettings.N # get size
    
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]).T.tolist(),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]).T.tolist(),
    )
    print(f"Received state {state}")
    # fix the case if there is not enough points
    if len(left) < 5 or len(right) < 5:
        print(left)
        yellow_line = LineString(left)
        blue_line = LineString(right)
        yellow_more = generate_N_points(yellow_line, 8)
        yellow_more = [(p.x, p.y) for p in yellow_more]
        blue_more = generate_N_points(blue_line, 8)
        blue_more = [(p.x, p.y) for p in blue_more]
        left, right = N_point_generator(yellow_more, blue_more, [state[:2]], N)
    else:
        left, right = N_point_generator(left, right, [state[:2]], N)
    return np.array(left), np.array(right)

def N_point_generator(left, right, traveled, N):
    # have a list of all_cones for refence
    all_cones = left.copy()
    all_cones.extend(right.copy())
    # figure out polygons
    # yellow_polygon, blue_polygon = find_local_polygons(all_cones, traveled, left)
    yellow_polygon, blue_polygon = find_local_polygons(all_cones, traveled, left, right)
    # figure out local paths
    local_yellow_path = remove_longest_edge_from_polygon(yellow_polygon)
    local_blue_path = remove_longest_edge_from_polygon(blue_polygon)
    # N point local paths
    N_local_yellow_path = LineString(generate_N_points(local_yellow_path, N))
    N_local_blue_path = LineString(generate_N_points(local_blue_path, N))
    outer_boundary_vertices = list(N_local_yellow_path.coords)
    inner_boundary_vertices = list(N_local_blue_path.coords)
    all_vertices = outer_boundary_vertices + inner_boundary_vertices
    # Voronoi diagram
    vor = Voronoi(all_vertices)
    # figure out medial line
    green_edges = get_medial_axis(N_local_yellow_path, N_local_blue_path, vor)
    new_green_edges = filter_medial_line_further(green_edges)
    # figure out closest points
    total_green_linestring = linemerge(new_green_edges)
    cpoml = closest_points_on_medial_line(outer_boundary_vertices, inner_boundary_vertices)
    cpoml_2 = closest_points_on_medial_line(cpoml, N_local_blue_path)
    # return stuff
    blue_boundary_vertices = [(point.x, point.y) for point in cpoml_2]
    return outer_boundary_vertices, blue_boundary_vertices

def closest_points_on_medial_line(points, medial_line):

    closest_points = []
    # Iterate over each point on the outer boundary
    for p in points:
    
        if isinstance(medial_line, MultiLineString):
            closest_points_on_multiline = []
            for line in medial_line.geoms:
                closest_point_on_line = line.interpolate(line.project(Point(p)))
                closest_points_on_multiline.append(closest_point_on_line)

            closest_point = min(closest_points_on_multiline, key=lambda t: (Point(p)).distance(t))
            closest_points.append(closest_point)
        else:
            medial_linestring = LineString(medial_line)
            closest_point_on_line = medial_linestring.interpolate(medial_linestring.project(Point(p)))
            closest_points.append(closest_point_on_line)
    
    return closest_points

def linestring_to_edges(linestring):
    edges = []
    coords = list(linestring.coords)
    num_coords = len(coords)
    for i in range(num_coords - 1):
        edges.append((coords[i], coords[i+1]))
    if linestring.is_closed:
        edges.append((coords[-1], coords[0]))
    return edges

def filter_medial_line_further(medial_edges):
    
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
    


def get_medial_axis(yellow_line, blue_line, vor):
    
    # filter edges
    non_boundary_edges = []
    for edge in vor.ridge_vertices:

        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_line = LineString([vor.vertices[edge[0]], vor.vertices[edge[1]]])
            if not edge_line.intersects(blue_line) and not edge_line.intersects(yellow_line):
                non_boundary_edges.append(edge_line) 
                
    all_cones = list(yellow_line.coords).copy()
    all_cones.extend(list(blue_line.coords).copy())
    cone_polygon = LineString(all_cones).convex_hull


    medial_edges = []
    for edge in vor.ridge_vertices:

        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_line = LineString([vor.vertices[edge[0]], vor.vertices[edge[1]]])
            if not edge_line.intersects(blue_line) and not edge_line.intersects(yellow_line):
                endpoint1 = Point(vor.vertices[edge[0]])
                endpoint2 = Point(vor.vertices[edge[1]])
                if cone_polygon.contains(endpoint1) and cone_polygon.contains(endpoint2):
                    medial_edges.append(edge_line)
                    
    # filter some more medial edges depending on if they are truly inside the path
    buffer_amount = yellow_line.distance(blue_line)
    polygon1 = yellow_line.buffer(buffer_amount)
    polygon2 = blue_line.buffer(buffer_amount)
    merged_polygon = polygon1.union(polygon2)

    medial_vertices = extract_points_from_linestrings(medial_edges)
    vertices_contained = []
    for point in medial_vertices:
        if merged_polygon.contains(Point(point)):
            vertices_contained.append(point)
    # filter edges that match points contained between liens
    true_medial_edges = []
    for edge in medial_edges:
        list_edge = list(edge.coords)
        first = list_edge[0]
        second = list_edge[1]
        if first in vertices_contained or second in vertices_contained:
            true_medial_edges.append(edge)
    
    return true_medial_edges


def distance_between_points(point1, point2):
    return np.sqrt(np.sum((np.array(point2) - np.array(point1)) ** 2))


def find_local_polygons(all_cones, traveled_points, yellow_points, blue_points):
        
    yellow_polygon = None
    blue_polygon = None
    
    current_diff_range = float('inf')
    
    for percentile_threshold in range(1, 100, 5):
        
        near_cones = find_near_cones(all_cones, traveled_points, percentile_threshold)
        
        filtered_yellow_points = []
        filtered_blue_points = []
        filtered_yellow_polygon = None
        filtered_blue_polygon = None
        
        for point in near_cones:
            if point in yellow_points:
                filtered_yellow_points.append(point)
            else:
                filtered_blue_points.append(point)
                
        if len(filtered_yellow_points) >= 4 and len(filtered_blue_points) >= 4:
            filtered_yellow_polygon, filtered_blue_polygon = find_polygons_from_points(yellow_points, blue_points)    
        
#         if yellow_polygon is None and filtered_yellow_polygon is not None and filtered_yellow_polygon.is_valid:
            
        if yellow_polygon is None and filtered_yellow_polygon is not None:
            
            all_traveled_contained = True
            for point in traveled_points:
                shapely_point = Point(point[0], point[1])
                if not shapely_point.within(filtered_yellow_polygon):
                    all_traveled_contained = False
                    break
                
            if all_traveled_contained:
                yellow_polygon = filtered_yellow_polygon
            
        if yellow_polygon is not None:
            min_x, min_y, max_x, max_y = yellow_polygon.bounds
            x_range = max_x - min_x
            y_range = max_y - min_y
            
#             if filtered_blue_polygon is not None and filtered_blue_polygon.is_valid:
            if filtered_blue_polygon is not None:
                
                bmin_x, bmin_y, bmax_x, bmax_y = filtered_blue_polygon.bounds
                blue_x_range = abs(bmax_x - bmin_x)
                blue_y_range = abs(bmax_y - bmin_y)
                
                new_diff = abs(x_range - blue_x_range) + abs(y_range - blue_y_range)
                
#                 if new_diff < current_diff_range and filtered_blue_polygon.is_valid:
                if new_diff < current_diff_range:
                    current_diff_range = new_diff
                    blue_polygon = filtered_blue_polygon
                
            elif filtered_blue_polygon is not None and (not filtered_blue_polygon.is_valid):
                break
    
    if yellow_polygon is None:
        yellow_polygon = filtered_yellow_polygon
        
    if blue_polygon is None:
        blue_polygon = filtered_blue_polygon
            
    return yellow_polygon, blue_polygon
            


def find_near_cones(cones, traveled_points, threshold_percentile=90):
    # Build KD-tree for traveled pointsy
    print("traveled_points: ", traveled_points)
    traveled_tree = KDTree(traveled_points)
    
    distances = []
    for cone in cones:
        _, idx = traveled_tree.query(cone)
        closest_traveled_point = traveled_points[idx]
        distances.append(distance_between_points(cone, closest_traveled_point))
    
    threshold_distance = np.percentile(distances, threshold_percentile)
    
    near_cones = []
    for i, cone in enumerate(cones):
        if distances[i] < threshold_distance:
            near_cones.append(cone)
    return near_cones


def remove_longest_edge_from_polygon(polygon):
    exterior_ring = polygon.exterior
    edges = LineString(exterior_ring).coords
    
    new_points = []
    longest_edge = None
    longest_edge_length = 0
    
    for i in range(len(edges) - 1):
        edge_start = edges[i]
        edge_end = edges[i + 1]
        edge_length = LineString([edge_start, edge_end]).length
        
        if edge_length > longest_edge_length:
            longest_edge_length = edge_length
            longest_edge = [edge_start, edge_end]
        
        new_points.append(edge_start)

    new_track = LineString(new_points)
    edge_to_remove = LineString(longest_edge)
    
    removal_i = -1
    
    for i in range(len(new_points)):
        if i == len(new_points) - 1:
            if new_points[i] == longest_edge[0] and new_points[0] == longest_edge[1]:
                removal_i = i
                break
        elif new_points[i] == longest_edge[0] and new_points[i + 1] == longest_edge[1]:
            removal_i = i
            break
    
    if removal_i == len(new_points) - 1:
        print(f"STUFFFFFF HEREEEE {new_points}")

        return LineString(new_points[0:-1])
    else:
        last_point = new_points[-1]
        first_point = new_points[0]

        stuff = [new_points[i+1:], [last_point, first_point], new_points[:i]]
        if len(new_points[i+1:]) == 0:
            stuff.remove(new_points[i+1:])
        if first_point == last_point:
            stuff.remove([last_point, first_point])
        if len(new_points[:i]) == 0:
            stuff.remove(new_points[:i])
        print(f"STUFFFFFF {stuff}")
        return LineString(sum(stuff, start=[]))

def generate_N_points(line, N):

    total_length = line.length

    step = total_length / (N - 1)
    equidistant_points = [line.interpolate(step * i) for i in range(N)]

    return equidistant_points

def extract_points_from_linestrings(linestrings):
    points = []
    for linestring in linestrings:
        coords = list(linestring.coords)
        points.extend(coords)
    return points
