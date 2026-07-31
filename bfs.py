# BFS

from collections import deque
import math

from solver_base import SolverResult


def find_path(G, orig_node, dest_node):
    #Dictionary to store the distance from the origin to each node
    unweighted_distances = {orig_node: 0, dest_node: math.inf}

    #Queue to store the nodes to be visited
    queue = deque([(0, orig_node)])
    visited = set([orig_node])
    path = {orig_node: [orig_node]}
    nodes_expanded = 0
    route = []

    #While the queue is not empty
    while queue:
        #pop next node in queue
        distance, node = queue.popleft()
        nodes_expanded += 1

        #If the distance is greater than the distance to the destination, continue
        if distance > unweighted_distances[dest_node]:
            continue
        #If the node is the destination, update distance to path
        if node == dest_node:
            route = path[node]
            return SolverResult(route=route, nodes_expanded=nodes_expanded, cost=distance, name="BFS")
        for neighbor in G.adj[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                new_distance = unweighted_distances[node] + 1
                unweighted_distances[neighbor] = new_distance
                queue.append((new_distance, neighbor))
                path[neighbor] = path[node] + [neighbor]
    # no path found
    return SolverResult(route=[], nodes_expanded=nodes_expanded, cost=math.inf, name="BFS")
