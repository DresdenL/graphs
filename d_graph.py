# Course: CS261 - Data Structures
# Author: Dresden Lee
# Assignment: 6
# Description: Directed graph with methods

import heapq
from collections import deque


class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        Adds a vertex to adj_matrix.
        """
        self.v_count += 1
        new_list = []
        for i in range(self.v_count):
            new_list.append(0)
        for e_list in self.adj_matrix:
            e_list.append(0)
        self.adj_matrix.append(new_list)
        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        Adds an edge to a vertex and checks if vertices are valid.
        """
        if src == dst or weight < 1:
            return
        if src < 0 or src >= self.v_count or dst < 0 or dst >= self.v_count:
            return

        self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        Removes an edge and checks if vertices are valid.
        """
        if src < 0 or src >= self.v_count or dst < 0 or dst >= self.v_count:
            return

        self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        Returns a list of vertices.
        """
        v_list = []
        for i in range(self.v_count):
            v_list.append(i)

        return v_list

    def get_edges(self) -> []:
        """
        returns a list of edges.
        """
        edge_list = []
        for i in range(self.v_count):
            for x in range(self.v_count):
                if self.adj_matrix[i][x] != 0:
                    edge_list.append((i, x, self.adj_matrix[i][x]))
        return edge_list

    def is_valid_path(self, path: []) -> bool:
        """
        Checks if path is valid, similar to logic of is_valid_path() from ud_graph.
        """
        if len(path) == 0:
            return True
        for i in range(len(path)):
            # check if vertex exists
            if path[i] < 0 or path[i] >= self.v_count:
                return False
            # check if we have reached end of path
            if i == len(path) - 1:
                return True
            # check if next vertex doesn't exist
            if path[i+1] < 0 or path[i + 1] >= self.v_count:
                return False
            # check if there is an edge between this vertex and next vertex
            if self.adj_matrix[path[i]][path[i + 1]] == 0:
                return False

    def dfs(self, v_start, v_end=None) -> []:
        """
        Depth first search. Help implementing this using the website below.
        https://www.tutorialspoint.com/data_structures_algorithms/depth_first_traversal.htm
        """
        the_deque = deque()
        visited_list = []
        # first check if v_start is in the graph
        if v_start < 0 or v_start >= self.v_count:
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
            # find next vertex to explore and append it to stack.
            current_min = None
            i = 0
            while i < self.v_count and current_min is None:
                if self.adj_matrix[current_vertex][i] > 0 \
                        and i not in visited_list:
                    current_min = i
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
        Breadth first search. Implemented with help using the link below.
        https://www.tutorialspoint.com/data_structures_algorithms/breadth_first_traversal.htm
        """
        visited_list = []
        the_deque = deque()

        if v_start < 0 or v_start >= self.v_count:
            return visited_list
        elif v_start == v_end:
            visited_list.append(v_start)
            return visited_list

        visited_list.append(v_start)
        current_vertex = v_start
        finished = False

        while not finished:
            for i in range(self.v_count):
                if self.adj_matrix[current_vertex][i] > 0 and i not in visited_list:
                    visited_list.append(i)
                    the_deque.append(i)
            if len(the_deque) == 0:
                finished = True
            else:
                current_vertex = the_deque[0]
                the_deque.popleft()
        return visited_list

    def has_cycle(self):
        """
        Checks if a cycle exists in the graph. Similar logic to has_cycle function of ud_graph.
        Help implementing this using link below. Uses the helper function, cycle_dfs()
        https://www.baeldung.com/cs/detecting-cycles-in-directed-graph
        """
        for i in range(self.v_count):
            is_cycle = self.cycle_dfs(i)
            if is_cycle:
                return True
        return False

    def dijkstra(self, src: int) -> []:
        """
        Function that uses Dijkstra's algorithm to find the shortest
        paths to each vertex given a starting vertex.
        Implemented using pseudocode from modules.
        """
        # tuple = (distance, vertex index)
        # initialize empty dijkstra list which will hold the shortest
        # path length for each vertex.
        d_list = {}
        # initialize priority queue
        pq = []
        heapq.heappush(pq, (0, src))
        # start at starting vertex
        while len(pq) > 0:
            the_tuple = heapq.heappop(pq)
            vertex = the_tuple[1]
            distance = the_tuple[0]
            if vertex not in d_list:
                d_list[vertex] = distance
                for i in range(self.v_count):
                    if self.adj_matrix[vertex][i] > 0:
                        heapq.heappush(pq, (distance + self.adj_matrix[vertex][i], i))
        dij_l = []
        for x in range(self.v_count):
            if x in d_list:
                dij_l.append(d_list[x])
            else:
                dij_l.append(float('inf'))
        return dij_l

    def cycle_dfs(self, v_start):
        """
        Helper function for has_cycle(). Checks if their is an edge to the starting vertex from
        descendants of starting vertex (except first descendant).
        """

        the_deque = deque()
        visited_list = []
        # first check if v_start is in the graph
        if v_start < 0 or v_start >= self.v_count:
            return visited_list
        the_deque.append(v_start)
        current_vertex = v_start
        visited_list.append(current_vertex)
        # visit adjacent node in ascending lexicographical order
        while len(the_deque) > 0:
            # find next vertex to explore and append it to stack.
            current_min = None
            if self.adj_matrix[current_vertex][v_start] > 0:
                return True
            i = 0
            while i < self.v_count and current_min is None:
                if self.adj_matrix[current_vertex][i] > 0 \
                        and i not in visited_list:
                    current_min = i
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
        return False


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
