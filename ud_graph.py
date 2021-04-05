# Course: CS 261
# Author: Dresden Lee
# Assignment: 6
# Description: Undirected graph with appropriate methods (add vertex, add edges, etc).

from collections import deque


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        Adds a new vertex to the graph
        """
        if str not in self.adj_list:
            self.adj_list[v] = []
        
    def add_edge(self, u: str, v: str) -> None:
        """
        Adds a new edge to the graph.
        If edges is already in graph, method does nothing.
        """
        if u == v:
            return
        if u in self.adj_list and v in self.adj_list:
            if u in self.adj_list[v]:
                return
            self.adj_list[u].append(v)
            self.adj_list[v].append(u)
        elif u in self.adj_list:
            self.adj_list[v] = [u]
            self.adj_list[u].append(v)
        elif v in self.adj_list:
            self.adj_list[u] = [v]
            self.adj_list[v].append(u)
        else:
            self.adj_list[u] = [v]
            self.adj_list[v] = [u]
        

    def remove_edge(self, v: str, u: str) -> None:
        """
        Removes an edge from the graph.
        """
        if v not in self.adj_list or u not in self.adj_list:
            return
        elif v not in self.adj_list[u]:
            return
        else:
            self.adj_list[v].remove(u)
            self.adj_list[u].remove(v)

    def remove_vertex(self, v: str) -> None:
        """
        Removes vertex and all connected edges.
        """
        if v not in self.adj_list:
            return
        else:
            self.adj_list.pop(v)
            for vertex in self.adj_list:
                if v in self.adj_list[vertex]:
                    self.adj_list[vertex].remove(v)

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order).
        """
        vertex_list = []
        for vertex in self.adj_list:
            vertex_list.append(vertex)
        return vertex_list

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order).
        """
        edges_list = []
        for vertex in self.adj_list:
            for edge in self.adj_list[vertex]:
                if (edge, vertex) not in edges_list:
                    edges_list.append((vertex, edge))
        return edges_list

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise.
        """
        if len(path) == 0:
            return True
        if len(path) == 1:
            if path[0] in self.adj_list:
                return True
            return False
        else:
            i = 0
            path_len = len(path)
            while i + 1 < path_len:
                # check if next vertex is an edge of current vertex
                if path[i] not in self.adj_list or path[i+1] not in self.adj_list:
                    return False
                elif path[i+1] not in self.adj_list[path[i]]:
                    return False
                i = i + 1
            return True

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order.

        (Help in implementing this algo by reading through this article:
        https://www.tutorialspoint.com/data_structures_algorithms/depth_first_traversal.htm)
        """
        # initialize deque to store visited vertices
        the_deque = deque()
        visited_list = []
        # first check if v_start is in the graph
        if v_start not in self.adj_list:
            return visited_list
        elif v_start == v_end:
            visited_list.append(v_start)
            return visited_list
        # add starting vertex to deque since it has been "visited" first.
        the_deque.append(v_start)
        current_vertex = v_start
        visited_list.append(current_vertex)
        # visit adjacent node in ascending lexicographical order
        while len(the_deque) > 0:
            if len(self.adj_list[current_vertex]) > 0:
                # find next vertex to explore and append it to stack.
                current_min = None
                i = 0
                while i < len(self.adj_list[current_vertex]):
                    if current_min is None:
                        if self.adj_list[current_vertex][i] not in visited_list:
                            current_min = self.adj_list[current_vertex][i]
                    elif self.adj_list[current_vertex][i] <= current_min and \
                            self.adj_list[current_vertex][i] not in visited_list:
                        current_min = self.adj_list[current_vertex][i]
                    i += 1
                if current_min is None:
                    the_deque.pop()
                    if len(the_deque) != 0:
                        current_vertex = the_deque[-1]
                # visit next vertex that has been identified and add to list
                else:
                    visited_list.append(current_min)
                    the_deque.append(current_min)
                    current_vertex = current_min
                    if current_min == v_end:
                        return visited_list
        return visited_list

    def bfs(self, v_start, v_end=None) -> []:
        """
        returns a list of vertices visited using breadth first search.
        Vertices picked in alphabetical order.
        Help in implementing this from:
        https://www.tutorialspoint.com/data_structures_algorithms/breadth_first_traversal.htm
        """
        visited_list = []
        the_deque = deque()

        if v_start not in self.adj_list:
            return visited_list
        elif v_start == v_end:
            visited_list.append(v_start)
            return visited_list

        visited_list.append(v_start)
        current_vertex = v_start
        finished = False

        while not finished:
            save_list = []
            for i in self.adj_list[current_vertex]:
                if i not in visited_list:
                    save_list.append(i)
            save_list.sort()
            for x in save_list:
                visited_list.append(x)
                if x == v_end:
                    return visited_list
                the_deque.append(x)
            if len(the_deque) == 0:
                finished = True
            else:
                current_vertex = the_deque[0]
                the_deque.popleft()
        return visited_list

    def count_connected_components(self):
        """
        Return number of connected components in the graph.
        Help in implementing this algorithm below:
        https://www.tutorialspoint.com/number-of-connected-components-in-an-undirected-graph-in-cplusplus
        """
        visited_nodes = []
        component_num = 0

        for vertex in self.adj_list:
            if vertex not in visited_nodes:
                if len(self.adj_list[vertex]) == 0:
                    visited_nodes.append(vertex)
                else:
                    new_list = self.dfs(vertex)
                    for i in new_list:
                        visited_nodes.append(i)
                component_num += 1
        return component_num

    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise
        help in implementing this algorithim below:
        https://www.baeldung.com/cs/cycles-undirected-graph

        For this, I edited my count_component function so that it would check
        if there is a cycle in each component of a graph. I also used a altered DFS function
        that kept track of edges.
        """
        visited_nodes = []
        component_num = 0

        for vertex in self.adj_list:
            if vertex not in visited_nodes:
                if len(self.adj_list[vertex]) == 0:
                    visited_nodes.append(vertex)
                else:
                    dfs_tuple = self.cycle_dfs(vertex)
                    if dfs_tuple[0] is True:
                        return True
                    else:
                        for i in dfs_tuple[1]:
                            visited_nodes.append(i)
                    if dfs_tuple[0] >= len(dfs_tuple[1]):
                        return True
                component_num += 1
        return False

    def cycle_dfs(self, v_start):
        """
        helper function for has_cycle(). Altered dfs method that keeps track of edges.
        """
        the_deque = deque()
        visited_list = []
        # first check if v_start is in the graph
        if v_start not in self.adj_list:
            return visited_list
        # add starting vertex to deque since it has been "visited" first.
        the_deque.append(v_start)
        current_vertex = v_start
        visited_list.append(current_vertex)
        num_of_edges = 0
        # visit adjacent node in ascending lexicographical order
        while len(the_deque) > 0:
            if len(self.adj_list[current_vertex]) > 0:
                # find next vertex to explore and append it to stack.
                current_min = None
                i = 0
                while i < len(self.adj_list[current_vertex]):
                    if current_min is None:
                        if self.adj_list[current_vertex][i] not in visited_list:
                            current_min = self.adj_list[current_vertex][i]
                    elif self.adj_list[current_vertex][i] <= current_min and \
                            self.adj_list[current_vertex][i] not in visited_list:
                        current_min = self.adj_list[current_vertex][i]
                    i += 1
                if current_min is None:
                    the_deque.pop()
                    if len(the_deque) != 0:
                        current_vertex = the_deque[-1]
                # visit next vertex that has been identified and add to list
                else:
                    visited_list.append(current_min)
                    the_deque.append(current_min)
                    current_vertex = current_min
        num_edges = 0
        for x in visited_list:
            num_edges += len(self.adj_list[x])

        edges_tuple = (int(num_edges/2), visited_list)
        return edges_tuple


if __name__ == '__main__':



    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
