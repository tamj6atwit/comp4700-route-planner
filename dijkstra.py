# Dijkstra

import heapq
import math

from solver_base import SolverResult


def find_path(G, orig_node, dest_node, weight="length"):

    # Priority queue stores (current cost, node)
    queue = [(0, orig_node)]

    # Shortest known distance to each node
    distances = {orig_node: 0}

    # Used to reconstruct path
    came_from = {}

    # Track visited nodes
    visited = set()

    nodes_expanded = 0

    while queue:
        current_cost, current = heapq.heappop(queue)

        # Skip if already processed
        if current in visited:
            continue

        visited.add(current)
        nodes_expanded += 1

        # Destination reached
        if current == dest_node:
            route = [current]

            while current in came_from:
                current = came_from[current]
                route.append(current)

            route.reverse()

            return SolverResult(
                route=route,
                nodes_expanded=nodes_expanded,
                cost=current_cost,
                name="Dijkstra"
            )

        # Check neighbors
        for neighbor, edge_data in G.adj[current].items():

            # osmnx graphs can have multiple edges between nodes
            edge_weight = min(
                data.get(weight, 1)
                for data in edge_data.values()
            )

            new_cost = current_cost + edge_weight

            # Found shorter path
            if new_cost < distances.get(neighbor, math.inf):
                distances[neighbor] = new_cost
                came_from[neighbor] = current

                heapq.heappush(
                    queue,
                    (new_cost, neighbor)
                )

    # No path found
    return SolverResult(
        route=[],
        nodes_expanded=nodes_expanded,
        cost=math.inf,
        name="Dijkstra"
    )