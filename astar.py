# A*
"""
A* search over an osmnx road-network graph, using a straight-line distance to the destination as the
heuristic. This is a simplification of the full great-circle (haversine).

Interface (see solver_base.py):
    find_path(G, orig_node, dest_node, weight="length") -> SolverResult
"""

import heapq
import math

from solver_base import SolverResult, MAX_SPEED_MPS

EARTH_RADIUS_M = 6371000.0


def flat_distance(lat1, lon1, lat2, lon2):
    lat_avg = math.radians((lat1 + lat2) / 2)
    dx = math.radians(lon2 - lon1) * math.cos(lat_avg)
    dy = math.radians(lat2 - lat1)
    return EARTH_RADIUS_M * math.sqrt(dx * dx + dy * dy)


def heuristic(G, node, dest_lat, dest_lon, weight):
    data = G.nodes[node]
    dist_m = flat_distance(data["y"], data["x"], dest_lat, dest_lon)
    if weight == "travel_time":
        # Convert straight-line meters into a lower-bound time (seconds) by
        # assuming the fastest possible speed in the graph, this keeps the
        # heuristic admissible (never overestimates the true travel time).
        return dist_m / MAX_SPEED_MPS
    return dist_m


def find_path(G, orig_node, dest_node, weight="length"):
    dest_lat = G.nodes[dest_node]["y"]
    dest_lon = G.nodes[dest_node]["x"]

    g_score = {orig_node: 0.0}
    came_from = {}
    visited = set()
    nodes_expanded = 0

    # heap entries: (f_score, tie_breaker, node)
    counter = 0
    open_heap = [(heuristic(G, orig_node, dest_lat, dest_lon, weight), counter, orig_node)]

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in visited:
            continue
        visited.add(current)
        nodes_expanded += 1

        if current == dest_node:
            route = [current]
            while current in came_from:
                current = came_from[current]
                route.append(current)
            route.reverse()
            return SolverResult(
                route=route,
                nodes_expanded=nodes_expanded,
                cost=g_score[dest_node],
                name="A*",
            )

        for neighbor, edge_data in G.adj[current].items():
            if neighbor in visited:
                continue
            edge_weight = min(d.get(weight, 1) for d in edge_data.values())
            tentative_g = g_score[current] + edge_weight

            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(G, neighbor, dest_lat, dest_lon, weight)
                counter += 1
                heapq.heappush(open_heap, (f_score, counter, neighbor))

    # no path found
    return SolverResult(route=[], nodes_expanded=nodes_expanded, cost=math.inf, name="A*")
