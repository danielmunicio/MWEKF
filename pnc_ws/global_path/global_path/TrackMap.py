import math
import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import Delaunay
from itertools import permutations
from shapely.geometry import Polygon, Point, LineString, MultiLineString
import networkx as nx

def distance_between_points(point1, point2):
    return np.sqrt(np.sum((np.array(point2) - np.array(point1)) ** 2))

def construct_total_points(yellow_points, blue_points):
    total_points = yellow_points.copy()
    total_points.extend(blue_points.copy())
    all_points = total_points
    total_points = np.array(total_points)
    return all_points, total_points

def remove_duplicates(edges):
    unique_edges = set()

    for edge in edges:
        # Check if the reversed edge is not in the set
        if (edge[1], edge[0]) not in unique_edges:
            unique_edges.add(edge)

    return list(unique_edges)

def categorize_edges(tri, yellow_points):
    yellow_edges = []
    blue_edges = []
    green_edges = []
    for simplex in tri.simplices:
        for i in range(3):
            edge = (simplex[i], simplex[(i+1)%3])
            if (edge[0] < len(yellow_points) and edge[1] < len(yellow_points)):
                yellow_edges.append(edge)
            elif (edge[0] >= len(yellow_points) and edge[1] >= len(yellow_points)):
                blue_edges.append(edge)
            else:
                green_edges.append(edge)
    return yellow_edges, blue_edges, green_edges

def get_triangles(tri):
    triangles = tri.simplices.tolist()
    return triangles

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

def keys_with_value_of_one(dictionary):
    return [key for key, value in dictionary.items() if value == 1]

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

def filter_triangles(tri, points):
    triangles = get_triangles(tri)
    true_triangles = []
    for t in triangles:
        t1, t2, t3 = t
        if t1 in points and t2 in points and t3 in points:
            true_triangles.append([t1, t2, t3])
    return true_triangles


def filter_green_triangles(tri, yellow_points, blue_points, all_points):
    triangles = get_triangles(tri)
    green_triangles = []
    for t in triangles:
        t1, t2, t3 = t
        t1_point, t2_point, t3_point = all_points[t1], all_points[t2], all_points[t3]
        in_blue_points = t1_point in blue_points or t2_point in blue_points or t3_point in blue_points
        in_yellow_points = t1_point in yellow_points or t2_point in yellow_points or t3_point in yellow_points
        
        if in_blue_points and in_yellow_points:
            green_triangles.append([t1, t2, t3])
    return green_triangles


def find_boundary_edges(triangles):
    triangle_counts = count_triangle_occurrences(triangles)
    boundary_edges = set(keys_with_value_of_one(triangle_counts))
    return boundary_edges

def graph_from_edges(edges):
    g = nx.Graph()
    g.add_edges_from(edges)
    return g

def vertices_in_edges(edges):
    return set([num for tup in edges for num in tup])

def make_cc_graphs(edges):
    # create the graph from the edges passed in
    figure_graph = graph_from_edges(edges)
    degrees = figure_graph.degree()
    # isolate the nodes which have a degree above 3
    nodes_of_degree_above_3 = [t[0] for t in degrees if t[1] > 3]
    higher_degree_edges = figure_graph.edges(nodes_of_degree_above_3)
    # find the connected_components of the subgraph
    higher_degree_graph = graph_from_edges(higher_degree_edges)
    high_degree_ccs = list(nx.connected_components(higher_degree_graph))
    # create the cc graphs + connecting_edges
    connecting_edges = edges.copy()
    cc_graphs = []
    for cc in high_degree_ccs:
        cc_edges = []
        for e in edges:
            if e[0] in cc and e[1] in cc:
                cc_edges.append(e)
                connecting_edges.remove(e)
        
        cc_graph = graph_from_edges(cc_edges)
        cc_graphs.append(cc_graph)
        
    return cc_graphs, connecting_edges

def combine_ccg_cycles(ccg_cycles, connecting_edges):
    all_the_edges = []
    for ccgc in ccg_cycles:
        all_the_edges.extend(ccgc)
    all_the_edges.extend(connecting_edges)
    return graph_from_edges(all_the_edges)

def find_cc_longest_cycles(cc_graphs, tri, total_points):
    cc_longest_cycles = []
    new_connecting_edges = []
    for ccg in cc_graphs:
        
        ccg_nodes = set(ccg.nodes())
        
        triangles = filter_triangles(tri, ccg_nodes)
        boundary_edges = find_boundary_edges(triangles)
        
        degrees = ccg.degree()
        nodes_of_degree_1 = [t[0] for t in degrees if t[1] < 2]
        if nodes_of_degree_1 != []:
            connections = ccg.edges(nodes_of_degree_1)
            new_connecting_edges.extend(connections)
        
        cc_longest_cycles.append(boundary_edges)
        
    return cc_longest_cycles, new_connecting_edges

def add_coordinates_back_into_edges(edges, all_points):
    true_edges = []
    for e in edges:
        first = all_points[e[0]]
        second = all_points[e[1]]
        true_e = (first, second)
        true_edges.append(true_e)
    return true_edges

def edges_to_connect_ccs_in_ccg(ccg, original_edges, all_points):
    
    def connect_two_subgraphs(sg1, sg2):  

        min_distance = float('inf')
        closest_nodes = None
        for node1 in sg1.nodes():
            for node2 in sg2.nodes():
                dist = distance_between_points(all_points[node1], all_points[node2])
                if dist < min_distance and ((node1, node2) in original_edges or (node2, node1) in original_edges):
                    min_distance = dist
                    closest_nodes = (node1, node2)
        if closest_nodes is not None:
            return [closest_nodes]
        else:
            return []
        
    subgraphs = []
    connected_components = list(nx.connected_components(ccg))
    for component in connected_components:
        subgraph = ccg.subgraph(component)
        subgraphs.append(subgraph)
    
    connecting_subcomponents = []
    
    if len(subgraphs) == 1:
        return []

    for i in range(len(subgraphs)):
        sg1 = subgraphs[i]
        sg2 = None
        if i == len(subgraphs) - 1:
            sg2 = subgraphs[0]
        else:
            sg2 = subgraphs[i + 1]
    
        connecting_subcomponents.extend(connect_two_subgraphs(sg1, sg2))
    
    return connecting_subcomponents
        
def track_shape(edges, total_tri, total_points, all_points, green_boundaries):
    cc_graphs, connecting_edges = make_cc_graphs(edges)
    
    cc_longest_cycles, new_connecting_edges = find_cc_longest_cycles(cc_graphs, total_tri, total_points)
    connecting_edges.extend(new_connecting_edges)
    
    for cclc in cc_longest_cycles:
        cclcg = graph_from_edges(cclc)
        ultra_new_connections = edges_to_connect_ccs_in_ccg(cclcg, edges, all_points)
        connecting_edges.extend(ultra_new_connections)
    
    filtered_graph = combine_ccg_cycles(cc_longest_cycles, connecting_edges)
    all_nodes = set(filtered_graph.nodes())
    filtered_edges = filtered_graph.edges(all_nodes)
    
    stuff_to_remove = []
    for edge in filtered_edges:
        if edge not in green_boundaries:
            stuff_to_remove.append(edge)
            
    for connect_edge in connecting_edges:
        if connect_edge in stuff_to_remove:
            stuff_to_remove.remove(connect_edge)
        else:
            reversed_connect_edge = (connect_edge[1], connect_edge[0])
            if reversed_connect_edge in stuff_to_remove:
                stuff_to_remove.remove(reversed_connect_edge)
    
    filtered_list = [x for x in filtered_edges if x not in stuff_to_remove]
    
    return filtered_list

def find_racetrack(yellow_points, blue_points):
    # construct triangulation
    all_points, total_points = construct_total_points(yellow_points, blue_points)
    tri = Delaunay(total_points)
    
    # categorize edges
    yellow_edges, blue_edges, green_edges = categorize_edges(tri, yellow_points) 
    
    # create representation of green triangles
    green_triangles = filter_green_triangles(tri, yellow_points, blue_points, all_points)
    green_boundaries = find_boundary_edges(green_triangles)
    
    # use information to find yellow and blue path edges
    yellow_path = track_shape(yellow_edges, tri, total_points, all_points, green_boundaries)
    blue_path = track_shape(blue_edges, tri, total_points, all_points, green_boundaries)
    
    yellow_path = remove_duplicates(yellow_path)
    blue_path = remove_duplicates(blue_path)
    
    yellow_path = add_coordinates_back_into_edges(yellow_path, all_points)
    blue_path = add_coordinates_back_into_edges(blue_path, all_points)
    
    return yellow_path, blue_path

def racetrack_to_multiline(yellow_path, blue_path):
    yellow_strings = [LineString([start, end]) for start, end in yellow_path]
    yellow_multiline = MultiLineString(yellow_strings)
    blue_strings = [LineString([start, end]) for start, end in blue_path]
    blue_multiline = MultiLineString(blue_strings)
    return yellow_multiline, blue_multiline

def lines_intersect(line1_p1, line1_p2, line2_p1, line2_p2):
    line1 = LineString([line1_p1, line1_p2])
    line2 = LineString([line2_p1, line2_p2])
    return line1.intersects(line2)

def line_intersects_edges(p1, p2, edges, excluding=[]):
    intersected = False
    for e in edges:
        if not (e[0] in excluding or e[1] in excluding):
            if lines_intersect(p1, p2, e[0], e[1]):
                intersected = True
                break
    return intersected

def find_longest_cycle(graph):
    longest_cycle = []
    for cycle in nx.simple_cycles(graph):
        if len(cycle) > len(longest_cycle):
            longest_cycle = cycle
    return longest_cycle

def edges_in_a_cycle(cycle):
    edges_in_longest_cycle = []
    for i in range(len(cycle) - 1):
        edges_in_longest_cycle.append([cycle[i], cycle[i+1]])
    if len(cycle) > 1:
        edges_in_longest_cycle.append([cycle[-1], cycle[0]])
    return edges_in_longest_cycle

def find_longest_simple_path(graph):
    longest_path = []

    # Function to perform DFS and find the longest path starting from a node
    def dfs_longest_path(node, visited, current_path):
        nonlocal longest_path

        # Mark the current node as visited
        visited.add(node)

        # Extend the current path with the current node
        current_path.append(node)

        # Update the longest path if the current path is longer
        if len(current_path) > len(longest_path):
            longest_path = current_path.copy()

        # Perform DFS on neighbors of the current node
        for neighbor in graph.neighbors(node):
            if neighbor not in visited:
                dfs_longest_path(neighbor, visited, current_path)

        # Backtrack: remove the current node from the current path
        current_path.pop()

        # Unmark the current node as visited (backtrack)
        visited.remove(node)

    # Perform DFS from each node to find the longest path
    for node in graph.nodes():
        dfs_longest_path(node, set(), [])

    return longest_path

def edges_in_a_path(path):
    edges_in_longest_path = []
    for i in range(len(path) - 1):
        edges_in_longest_path.append([path[i], path[i+1]])
    return edges_in_longest_path