import numpy as np
from scipy.spatial import Delaunay
import math
from shapely.geometry import Polygon, LineString
from itertools import permutations
from .Graph import Graph

def find_polygons_from_points(yellow_points, blue_points):
    total_points = construct_total_points(yellow_points, blue_points)[1]
    yellow_edges, blue_edges = find_total_shape(yellow_points, blue_points)
    yellow_polygon = create_polygon_from_edges(yellow_edges, total_points)
    blue_polygon = create_polygon_from_edges(blue_edges, total_points)
    return yellow_polygon, blue_polygon

def sort_edges_by_start_point(edges):
    return sorted(edges, key=lambda edge: min(edge))

def create_polygon_from_edges(edges, total_points):
    edges = sort_edges_by_start_point(edges)
    polygon = []
    
    # Initialize the polygon with the first edge
    polygon.append(edges[0])
    del edges[0]
    
    while edges:
        found = False
        for i, edge in enumerate(edges):
            if edge[0] == polygon[-1][-1]:  # If the starting point of the edge matches the last point of the polygon
                polygon.append(edge)
                del edges[i]
                found = True
                break
            elif edge[-1] == polygon[-1][-1]:  # If the ending point of the edge matches the last point of the polygon
                polygon.append(edge[::-1])  # Reverse the edge to match the direction of the polygon
                del edges[i]
                found = True
                break
        if not found:
            break  # If no matching edge is found, exit the loop
    
    # Check if the polygon is closed
    if polygon[0][0] == polygon[-1][-1]:
    
        stuff_in_order = []
        for e in polygon:
            the_point = total_points[e[0]]
            stuff_in_order.append(the_point)
        return Polygon(stuff_in_order)
    
    else:
        return None  # If the polygon is not closed, return None


def find_total_shape(yellow_points, blue_points):
    # np arrays for yellow, blue, and all points
    all_points, total_points = construct_total_points(yellow_points, blue_points)
    np_yellow = np.array(yellow_points)
    np_blue = np.array(blue_points)
    # triangulations
    tri = Delaunay(total_points)
    yellow_tri = Delaunay(np_yellow)
    blue_tri = Delaunay(np_blue)
    # categorize edges
    yellow_edges, blue_edges, green_edges = categorize_edges(tri, len(yellow_points))
    purple_edges = find_purple_edges(tri, blue_points, total_points)
    # remove duplicates
    yellow_edges = remove_duplicates(yellow_edges)
    blue_edges = remove_duplicates(blue_edges)
    purple_edges = remove_duplicates(purple_edges)
    green_edges = remove_duplicates(green_edges)
    # find new_yellow_edges which only intersect greens
    yellow_triangles = get_triangles(yellow_tri)
    yellow_tri_edges = get_edges_from_triangles(yellow_triangles)
    new_yellow_edges = new_edges_from_greens(yellow_tri_edges, purple_edges, yellow_points, total_points, existing_edges=yellow_edges)
    # find new blue edges which only intersect greens
    blue_triangles = get_triangles(blue_tri)
    blue_tri_edges = get_edges_from_triangles(blue_triangles)
    # offset the blue_edges
    true_blue_tri_edges = []
    offset = len(yellow_points)
    for bte in blue_tri_edges:
        bte1, bte2 = bte
        new_bte = (bte1 + offset, bte2 + offset)
        true_blue_tri_edges.append(new_bte)
    new_blue_edges = new_edges_from_greens(true_blue_tri_edges, yellow_edges, all_points, total_points, existing_edges=blue_edges)
    
    yellow_edges.extend(new_yellow_edges)
    yellow_edges = remove_duplicates(yellow_edges)
    blue_edges.extend(new_blue_edges)
    blue_edges = remove_duplicates(blue_edges)
    
    # find connecting yellow edges
    connecting_yellows = find_connecting_yellows(yellow_tri_edges, yellow_edges, green_edges, purple_edges, yellow_points, total_points)
    all_yellow_connections = []
    for cc in connecting_yellows:
        all_yellow_connections.extend(cc)
    yellow_edges.extend(all_yellow_connections)
    yellow_edges = remove_duplicates(yellow_edges)
    
    # remove blues which intersect connecting yellows
    blues_to_remove = find_blues_intersecting_yellows(blue_edges, all_yellow_connections, total_points)
    print("blues to remove: ", blues_to_remove)
    for elem in blues_to_remove:
        if elem in blue_edges:
            blue_edges.remove(elem)
    # trim the yellows
    yellow_edges = trim_figure(yellow_tri, yellow_edges, total_points)
    yellow_edges = remove_duplicates(yellow_edges)
    # trim the blues
    blue_edges = find_final_blue_boundary(blue_tri, blue_edges, total_points, len(yellow_points))
    blue_edges = remove_duplicates(blue_edges)
    # return
    return yellow_edges, blue_edges

def find_final_blue_boundary(blue_tri, blue_edges, total_points, offset_from_yellow):
    num_points = len(total_points)
    true_triangles = []
    triangles = get_triangles(blue_tri)
    for t in triangles:
        t1, t2, t3 = t
        t1 += offset_from_yellow
        t2 += offset_from_yellow
        t3 += offset_from_yellow
        # check if all three edges are present in blue_edges
        first_bool = (t1, t2) in blue_edges or (t2, t1) in blue_edges
        second_bool = (t1, t3) in blue_edges or (t3, t1) in blue_edges
        third_bool = (t2, t3) in blue_edges or (t3, t2) in blue_edges
        if first_bool and second_bool and third_bool:
            true_triangles.append([t1, t2, t3])

    triangle_counts = count_triangle_occurrences(true_triangles)
    boundary_edges = set(keys_with_value_of_one(triangle_counts))
    return boundary_edges

def trim_figure(tri, figure_edges, total_points, provided_triangles=None):
    # initialize stuff
    num_points = len(total_points)
#     graph = triangulation_graph(tri, num_points)
    triangles = get_triangles(tri)
    if provided_triangles:
        triangles = provided_triangles
    required_edges = []
    required_triangles = []
    # iteratively remove edges
    while True:
        # update boundaries
        triangle_counts = count_triangle_occurrences(triangles)
        boundary_edges = set(keys_with_value_of_one(triangle_counts))
        longest_edge = find_edge_with_longest_distance(boundary_edges, total_points, required_edges)
        # check if graph will still be in good shape
        # remove all three edges of longest edge triangle
        # add back edges that are not longest edge
        # check all the boundaries in this graph + the two added back edges form a polygon
        if longest_edge is not None:
            # get all the vertices of the triangle that we want to remove
            t = delete_triangle(triangles, longest_edge)
            if t != []:
                t1, t2, t3 = t[0]
            else:
                break
            t_set = {t1, t2, t3}
            long_set = set(longest_edge)
            third_vertex = (t_set - long_set).pop()
            # check if all of the edges are yellow edges
            bool_one = edge_in_edge_list(longest_edge, figure_edges)
            bool_two = edge_in_edge_list((longest_edge[0], third_vertex), figure_edges)
            bool_three = edge_in_edge_list((longest_edge[1], third_vertex), figure_edges)
            if not (bool_one and bool_two and bool_three):
                triangles.append([t1, t2, t3])
                required_edges.append(longest_edge)
                continue
            # remove all edges of triangle from boundaries
            boundary_edges.discard((t1, t2))
            boundary_edges.discard((t2, t1))
            boundary_edges.discard((t1, t3))
            boundary_edges.discard((t3, t1))
            boundary_edges.discard((t2, t3))
            boundary_edges.discard((t3, t2))
            # create boundaries graph
            boundary_edges.add((longest_edge[0], third_vertex))
            boundary_edges.add((longest_edge[1], third_vertex))
            bgraph = boundary_graph(list(boundary_edges), num_points)
            bvertices = vertices_in_edges(list(boundary_edges))
            # check if boundaries form a polygon
            if bgraph.forms_polygon(bvertices):
#                 graph.remove_edge(longest_edge[0], longest_edge[1])
                pass
            else:
                # that edge was not safe to remove
                triangles.append([t1, t2, t3])
                required_edges.append(longest_edge)
        else:
            break
            
    triangle_counts = count_triangle_occurrences(triangles)
    boundary_edges = set(keys_with_value_of_one(triangle_counts))
        
    return boundary_edges

def edge_in_edge_list(edge, edge_list):
    e1, e2 = edge
    return edge in edge_list or (e2, e1) in edge_list

def find_blues_intersecting_yellows(blue_edges, intersecting_yellows, total_points):
    to_remove = []
    for b_edge in blue_edges:
        b1, b2 = b_edge
        b1 = total_points[b1]
        b2 = total_points[b2]
        for y_edge in intersecting_yellows:
            y1, y2 = y_edge
            y1 = total_points[y1]
            y2 = total_points[y2]
            if lines_intersect(y1, y2, b1, b2):
                to_remove.append(b_edge)
    return to_remove

def find_connecting_yellows(yellow_tri_edges, yellow_edges, green_edges, purple_edges, yellow_points, total_points):
    all_the_connections = []
    # construct yellow_graph to find connected components
    yellow_graph = boundary_graph(yellow_edges, len(yellow_points))
    ccs = yellow_graph.connected_components()
    ccs = sorted(ccs, key=len)
    # loop through each connected component
    for i in range(len(ccs) - 1):
        if ccs[i] == []:
            continue
        vertices_in_cc = vertices_in_edges(ccs[i])
        # find all the green edges connected to yellow vertices
        selected_greens = []
        for g_edge in green_edges:
            g1, g2 = g_edge
            if g1 in vertices_in_cc or g2 in vertices_in_cc:
                selected_greens.append(g_edge)
        # find the green vertices
        green_vertices = vertices_in_edges(selected_greens)
        # find all the purple edges involving the selected green vertices
        selected_purples = []
        for p_edge in purple_edges:
            p1, p2 = p_edge
            if p1 in green_vertices or p2 in green_vertices:
                selected_purples.append(p_edge)
        print("selected_purples: ", selected_purples)
        # find the longest purple edge surrounding the connected component
        long_dist = 0
        longest_purple = None
        for p_edge in selected_purples:
            p1, p2 = p_edge
            p1 = total_points[p1]
            p2 = total_points[p2]
            dist = distance_between_points(p1, p2)
            print("dist: ", dist)
            if dist > long_dist:
                long_dist = dist
                longest_purple = p_edge
        # find the purples which intersect the lines from each yellow_vertex to the longest_purple
        avg_x = 0
        avg_y = 0
        for yv in vertices_in_cc:
            yp =  yellow_points[yv]
            avg_x += yp[0]
            avg_y += yp[1]
        avg_x /= len(vertices_in_cc)
        avg_y /= len(vertices_in_cc)
        print("longest: ", longest_purple)
        first_purple = total_points[longest_purple[0]]
        second_purple = total_points[longest_purple[1]]
        intersecting_purples = same_direction(first_purple, second_purple, (avg_x, avg_y), purple_edges, total_points)
        intersecting_purples = remove_duplicates(intersecting_purples)
        # find edges from yellow_tri which intersect long_purple and selected_purple
        new_yellows = []
        for edge in yellow_tri_edges:
            e1, e2 = edge
            e1 = yellow_points[e1]
            e2 = yellow_points[e2]
            num_intersected = 0
            for p_edge in intersecting_purples:
                p1, p2 = p_edge
                p1 = total_points[p1]
                p2 = total_points[p2]
                if lines_intersect(e1, e2, p1, p2):
                    num_intersected += 1
            if num_intersected == len(intersecting_purples):
                new_yellows.append(edge)
        all_the_connections.append(new_yellows)
    return all_the_connections

def same_direction(p1, p2, yp, line_segments, total_points):

    first_angle = polar_angle_from_origin(yp, p1)
    second_angle = polar_angle_from_origin(yp, p2)
    mid_angle = polar_angle_from_origin(yp, midpoint(p1, p2))
    
    angles = sorted([first_angle, second_angle])
    result = []
    for segment in line_segments:
        s1, s2 = segment
        s1 = total_points[s1]
        s2 = total_points[s2]
        as1 = polar_angle_from_origin(yp, s1)
        as2 = polar_angle_from_origin(yp, s2)
        sorted_as = sorted([as1, as2])
        if sorted_as[0] < mid_angle and mid_angle < sorted_as[1]:
            
            new_s = midpoint(s1, s2)
            seg_angle = polar_angle_from_origin(yp, new_s)
            if angles[0] < seg_angle and seg_angle < angles[1]:
                result.append(segment)

    return result

def polar_angle_from_origin(origin, point):
    dy = point[1] - origin[1]
    dx = point[0] - origin[0]
    return math.atan2(dy, dx)

def midpoint(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2

    return mid_x, mid_y

def remove_duplicates(edges):
    unique_edges = set()

    for edge in edges:
        # Check if the reversed edge is not in the set
        if (edge[1], edge[0]) not in unique_edges:
            unique_edges.add(edge)

    return list(unique_edges)

def new_edges_from_greens(tri_edges, purple_edges, colored_points, total_points, existing_edges=[]):
    new_edges = []
    for edge in tri_edges:
        if edge in existing_edges:
            continue
        e1, e2 = edge
        e1 = colored_points[e1]
        e2 = colored_points[e2]
        intersected_purple = False
        for p_edge in purple_edges:
            p1, p2 = p_edge
            p1 = total_points[p1]
            p2 = total_points[p2]
            if lines_intersect(e1, e2, p1, p2):
                intersected_purple = True
                break;
        if not intersected_purple:
            new_edges.append(edge)
    new_edges = list(set(new_edges))
    return new_edges

def find_purple_edges(tri, blue_points, total_points):
    blue_triangles = filter_triangles_for_blues(tri, blue_points, total_points)
    triangle_counts = count_triangle_occurrences(blue_triangles)
    blue_boundary_edges = set(keys_with_value_of_one(triangle_counts))
    return blue_boundary_edges

def filter_triangles_for_blues(tri, blue_points, total_points):
    # find the intial stuff
    triangles = get_triangles(tri)
    blue_triangles = []
    for t in triangles:
        t1, t2, t3 = t
        t1 = total_points[t1]
        t2 = total_points[t2]
        t3 = total_points[t3]
        
        if (tuple(t1) in blue_points) and (tuple(t2) in blue_points) and (tuple(t3) in blue_points):
            blue_triangles.append(t)

    return blue_triangles

def categorize_edges(tri, yellow_num):
    yellow_edges = []
    blue_edges = []
    green_edges = []
    for simplex in tri.simplices:
        for i in range(3):
            edge = (simplex[i], simplex[(i+1)%3])
            if (edge[0] < yellow_num and edge[1] < yellow_num):
                yellow_edges.append(edge)
            elif (edge[0] >= yellow_num and edge[1] >= yellow_num):
                blue_edges.append(edge)
            else:
                green_edges.append(edge)
    return yellow_edges, blue_edges, green_edges

def lines_intersect(line1_p1, line1_p2, line2_p1, line2_p2):
    line1 = LineString([line1_p1, line1_p2])
    line2 = LineString([line2_p1, line2_p2])
    return line1.intersects(line2)

def vertices_in_edges(edges):
    return set([num for tup in edges for num in tup])

def get_edges_from_triangles(triangles):
    edges = set()
    for triangle in triangles:
        # Extract edges from the triangle
        edge1 = tuple(sorted((triangle[0], triangle[1])))
        edge2 = tuple(sorted((triangle[1], triangle[2])))
        edge3 = tuple(sorted((triangle[2], triangle[0])))
        # Add edges to the set
        edges.update([edge1, edge2, edge3])
    
    # Convert set of edges to a list
    edges_list = list(edges)
    return edges_list

def delete_triangle(triangles, edge):
    triangles_to_remove = []
    for t in triangles:
        if set(edge) <= set(t):
            triangles_to_remove.append(t)
    for t in triangles_to_remove:
        triangles.remove(t)
    return triangles_to_remove

def find_edge_with_longest_distance(edges, points, required_edges=[]):
    best_edge_distance = 0
    best_edge = None
    for edge in edges:
        p1 = points[edge[0]]
        p2 = points[edge[1]]
        distance = distance_between_points(p1, p2)  
        if distance > best_edge_distance and (edge not in required_edges):
            best_edge_distance = distance
            best_edge = edge
    return best_edge

def boundary_graph(edges, num_vertices):
    g = Graph(num_vertices)
    for e in edges:
        g.add_edge(e[0], e[1])
    return g

def triangulation_graph(tri, num_vertices):
    graph_edges = get_tri_edges(tri)
    g = Graph(num_vertices)
    for e in graph_edges:
        g.add_edge(e[0], e[1])
    return g

def keys_with_value_of_one(dictionary):
    return [key for key, value in dictionary.items() if value == 1]

def count_triangle_occurrences(triangles):
    counts = {}
    for triangle in triangles:
        # Store edges of the triangle in a set to avoid double counting
        triangle_edges = set()
        for edge in permutations(triangle, 2):
            sorted_edge = tuple(sorted(edge))
            # Add edge to counts if it hasn't been encountered in this triangle
            if sorted_edge not in triangle_edges:
                if sorted_edge in counts:
                    counts[sorted_edge] += 1
                else:
                    counts[sorted_edge] = 1
                # Add edge to set of triangle_edges
                triangle_edges.add(sorted_edge)
    return counts

def get_triangles(tri):
    triangles = tri.simplices.tolist()
    return triangles

def get_tri_edges(tri):
    
    edges = set()
    # Loop through each simplex (triangle)
    for simplex in tri.simplices:
        # Add all edges of the simplex to the set
        for i in range(3):
            edge = tuple(sorted((simplex[i], simplex[(i + 1) % 3])))
            edges.add(edge)

    edges_list = list(edges)
    return edges_list

def construct_total_points(yellow_points, blue_points):
    total_points = yellow_points.copy()
    total_points.extend(blue_points)
    all_points = total_points
    total_points = np.array(total_points)
    return all_points, total_points

def distance_between_points(point1, point2):
    return np.sqrt(np.sum((np.array(point2) - np.array(point1)) ** 2))
