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


    def forms_polygon(self, boundary_vertices):
        
        if not self.has_cycle():
            return False
        
        for v in range(self.num_vertices):
            if v in boundary_vertices:
                degree = len(self.adjacency_list[v])
                if degree != 2:
                    return False
        
        return True
    
    def degrees(self, vertices):
        degrees = {}
        for v in vertices:
            degrees[v] = len(self.adjacency_list[v])
        return degrees