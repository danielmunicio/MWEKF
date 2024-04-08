from .global_opt_settings import GlobalOptSettings
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import nearest_points, linemerge


from feb_msgs.msg import Map

class Graph:
    def __init__(self, num_vertices):
        self.num_vertices = num_vertices
        self.adjacency_list = {vertex: [] for vertex in range(0, num_vertices)}

    def add_edge(self, vertex1, vertex2):
        self.adjacency_list[vertex1].append(vertex2)
        self.adjacency_list[vertex2].append(vertex1)
    
    def remove_edge(self, vertex1, vertex2):
        self.adjacency_list[vertex1].remove(vertex2)
        self.adjacency_list[vertex2].remove(vertex1)

    def display(self):
        for vertex in self.adjacency_list:
            print(vertex, "->", " -> ".join(map(str, self.adjacency_list[vertex])))
            
    def is_fully_connected(self):
        visited = set()

        def dfs(node):
            visited.add(node)
            for neighbor in self.adjacency_list[node]:
                if neighbor not in visited:
                    dfs(neighbor)

        dfs(0)

        return len(visited) == self.num_vertices
    
    def get_edges(self):
        edges = []
        for vertex in self.adjacency_list:
            for neighbor in self.adjacency_list[vertex]:
                if (neighbor, vertex) not in edges:  # Avoid duplicates in undirected graph
                    edges.append((vertex, neighbor))
        return edges
    
    def has_cycle(self):
        
        visited = set()
        
        def helper(vertex, parent):
            visited.add(vertex)
            for neighbor in self.adjacency_list[vertex]:
                if neighbor not in visited:
                    if helper(neighbor, vertex):
                        return True
                elif parent != neighbor:
                    return True
            return False
        
        
        for vertex in range(self.num_vertices):
            if vertex not in visited:
                if helper(vertex, -1):
                    return True
        return False
    
    def connected_components(self):
        visited = set()
        components = []

        def dfs(node, component):
            visited.add(node)
            component.append(node)
            for neighbor in self.adjacency_list[node]:
                if neighbor not in visited:
                    dfs(neighbor, component)

        for vertex in range(self.num_vertices):
            if vertex not in visited:
                component = []
                dfs(vertex, component)
                components.append(component)

        # Generate edges for each component
        result = []
        for component in components:
            component_edges = []
            for vertex in component:
                for neighbor in self.adjacency_list[vertex]:
                    if neighbor in component and (vertex, neighbor) not in component_edges:
                        component_edges.append((vertex, neighbor))
            result.append(component_edges)

        return result


    def forms_polygon(self):
        
        if not self.has_cycle():
            return False
        
        for v in range(self.num_vertices):
            degree = len(self.adjacency_list[v])
            if degree != 2:
                return False
        
        return True
    
    def degrees(self, vertices):
        degrees = {}
        for v in vertices:
            degrees[v] = len(self.adjacency_list[v])
        return degrees

def ConeOrdering(msg: Map):
    """get cones from message and call the cone ordering algorithm and return the results

    Args:
        msg (Cones): the message that we received

    Returns:
        tuple[ndarray(2, N), ndarray(2, N)]: pairs of points on the track boundary
    """
    N = GlobalOptSettings.N # get size
    left, right = (
        np.array([list(msg.left_cones_x), list(msg.left_cones_y)]),
        np.array([list(msg.right_cones_x), list(msg.right_cones_y)]),
    )
    left, right = cone_ordering_algorithm(left, right, N)
    return left, right

def cone_ordering_algorithm(left, right, N):
    """
    even more dummy placeholder algorithm
    """
    order(left)
    order(right)
    l = interp(left)(np.linspace(0.0, 1.0, N, endpoint=False))
    r = interp(right)(np.linspace(0.0, 1.0, N, endpoint=False))

    return l, r

def interp(points):

    dx = np.linalg.norm(np.diff(points, axis=0), axis=1)
    dists = np.cumsum(dx)
    dists = np.hstack([np.array([0.0]), dists])
    
    def interpolate(x):
        x = x*dists[-1]
        i = np.where(x<dists)[0][0]
        t=(x-dists[i-1])/dx[i-1]

        return points[i-1]*(1-t) + points[i]*t
    return lambda x: np.array([interpolate(i) for i in x])

def order(cones):
    """
    orders the input array by selecting the nearest cone. runs in-place, assumes first cone is correct

    Args:
        cones (ndarray): array of shape (n, 2) with cone points. will be modified in-place. first cone must be correct.
    """
    for i in range(len(cones)-2):
        mindex = np.argmin(np.linalg.norm(cones[i+1:]-cones[i], axis=1))+i+1
        cones[i+1], cones[mindex] = np.copy(cones[mindex]), np.copy(cones[i+1])

def getMedial(left, right, N): 
    yellow_polygon = Polygon(left)
    blue_polygon = Polygon(right)

    new_yellow_points = generate_N_points(yellow_polygon, N)
    new_blue_points = generate_N_points(blue_polygon, N)

    y_polygon = Polygon(new_yellow_points)
    b_polygon = Polygon(new_blue_points)

    new_outer_boundary_vertices = list(y_polygon.exterior.coords)
    new_inner_boundary_vertices = list(b_polygon.exterior.coords)
    all_vertices = new_outer_boundary_vertices + new_inner_boundary_vertices

    dense_vor = Voronoi(all_vertices)

    yellow_boundary = LineString(y_polygon.exterior)
    blue_boundary = LineString(b_polygon.exterior)

    # Extract Medial Axis

    yellow_boundary = LineString(y_polygon.exterior)
    blue_boundary = LineString(b_polygon.exterior)

    # filter edges
    non_boundary_edges = []
    for edge in dense_vor.ridge_vertices:
    
        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_line = LineString([dense_vor.vertices[edge[0]], dense_vor.vertices[edge[1]]])
            if not edge_line.intersects(blue_boundary) and not edge_line.intersects(yellow_boundary):
                non_boundary_edges.append(edge_line)
        

    medial_edges = []
    for edge in dense_vor.ridge_vertices:
        
        # Both indices are valid
        if edge[0] >= 0 and edge[1] >= 0:
            edge_line = LineString([dense_vor.vertices[edge[0]], dense_vor.vertices[edge[1]]])
            if not edge_line.intersects(blue_boundary) and not edge_line.intersects(yellow_boundary):
                
                endpoint1 = Point(dense_vor.vertices[edge[0]])
                endpoint2 = Point(dense_vor.vertices[edge[1]])
                if not b_polygon.contains(endpoint1) and not b_polygon.contains(endpoint2) \
                    and y_polygon.contains(endpoint1) and y_polygon.contains(endpoint2):
                    medial_edges.append(edge_line)

    if (len(medial_edges) == 0):
        getMedial(left, right, N + 50)
    else:
        medial_points = list(set(extract_points_from_linestrings(medial_edges)))
        medial_dict = {}

        idx = 0
        for point in medial_points:
            medial_dict[point] = idx
            idx += 1


        middle_edges = []
        for lstring in medial_edges:
            middle_edges.extend(linestring_to_edges(lstring))
        
        graph_edges = []

        graph = Graph(len(medial_points))
        for e in middle_edges:
            index_of_e0 = medial_dict[e[0]]
            index_of_e1 = medial_dict[e[1]]
            graph.add_edge(index_of_e0, index_of_e1)

        longest_cycle = find_longest_cycle(graph)
        print("Longest cycle:", longest_cycle)
        print(len(longest_cycle))

        points_in_longest_cycle = []
        for i in longest_cycle:
            points_in_longest_cycle.append(medial_points[i])

        mid_polygon = Polygon(points_in_longest_cycle)
        cpoml = closest_points_on_medial_line(new_yellow_points, mid_polygon)

        # cpoml_2 = closest_points_on_medial_line(new_blue_points, mid_polygon)
        cpoml_2 = closest_points_on_medial_line(all_closest_points, b_polygon)
        # print(cpoml_2)
        all_points_2 = list(cpoml_2.keys())
        all_closest_points_2 = [cpoml_2[point][0] for point in all_points_2]
        all_points = list(cpoml.keys())
        all_closest_points = [cpoml[point][0] for point in all_points]

        #Final return values
        N_yellows = new_yellow_points
        N_blues = all_closest_points_2

        return N_yellows, N_blues



def extract_points_from_linestrings(linestrings):
    points = []
    for linestring in linestrings:
        coords = list(linestring.coords)
        points.extend(coords)
    return points

def vertices_in_edges(edges):
    return set([num for tup in edges for num in tup])

def dfs_cycle(graph, start, current, visited, path, longest_cycle):
    visited.add(current)
    path.append(current)

    for neighbor in graph.adjacency_list[current]:
        if neighbor not in visited:
            dfs_cycle(graph, start, neighbor, visited, path, longest_cycle)
        elif neighbor == start and len(path) > 2:
            cycle_length = len(path)
            if cycle_length > len(longest_cycle):
                longest_cycle[:] = path[:]
    path.pop()
    visited.remove(current)


def find_longest_cycle(graph):
    longest_cycle = []
    for vertex in graph.adjacency_list:
        dfs_cycle(graph, vertex, vertex, set(), [], longest_cycle)
    return longest_cycle


def linestring_to_edges(linestring):
    edges = []
    coords = list(linestring.coords)
    num_coords = len(coords)
    for i in range(num_coords - 1):
        edges.append((coords[i], coords[i+1]))
    # If the LineString is closed (represents a polygon), add the edge connecting the last vertex to the first vertex
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