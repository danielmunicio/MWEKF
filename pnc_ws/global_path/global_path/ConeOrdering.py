from .global_opt_settings import GlobalOptSettings
import numpy as np
from scipy.spatial import Voronoi

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
