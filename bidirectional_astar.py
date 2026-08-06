# Bidirectional A*
"""
Bidirectional A*: search from origin and destination at the same time.
Both sides use the same straight-line heuristic as uni-directional A*.

Stops once a complete path cost `best` is known and the combined lower
bound from both frontiers (min_f_forward + min_f_backward) meets or
exceeds it — the standard correct termination test for bidirectional
best-first search.

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


def heuristic(G, node, goal_lat, goal_lon, weight):
    data = G.nodes[node]
    dist_m = flat_distance(data["y"], data["x"], goal_lat, goal_lon)
    if weight == "travel_time":
        return dist_m / MAX_SPEED_MPS
    return dist_m


def find_path(G, orig_node, dest_node, weight="length"):
    if orig_node == dest_node:
        return SolverResult([orig_node], 0, 0.0, "Bidirectional A*")

    dest_lat, dest_lon = G.nodes[dest_node]["y"], G.nodes[dest_node]["x"]
    orig_lat, orig_lon = G.nodes[orig_node]["y"], G.nodes[orig_node]["x"]

    g_f, g_b = {orig_node: 0.0}, {dest_node: 0.0}
    came_f, came_b = {}, {}
    closed_f, closed_b = set(), set()

    counter = 0
    open_f = [(heuristic(G, orig_node, dest_lat, dest_lon, weight), 0.0, counter, orig_node)]
    counter += 1
    open_b = [(heuristic(G, dest_node, orig_lat, orig_lon, weight), 0.0, counter, dest_node)]

    best, meet = math.inf, None
    nodes_expanded = 0

    def edge_cost(edge_data):
        return min(d.get(weight, 1) for d in edge_data.values())

    def meet_cost(node):
        nonlocal best, meet
        if node in g_f and node in g_b:
            cost = g_f[node] + g_b[node]
            if cost < best:
                best, meet = cost, node

    def top_f(heap, g, closed):
        """Pop stale entries off the top and return the valid min f (heap left intact otherwise)."""
        while heap:
            f, g_pop, _, node = heap[0]
            if node in closed or g_pop > g.get(node, math.inf):
                heapq.heappop(heap)
                continue
            return f
        return math.inf

    while open_f and open_b:
        f_min = top_f(open_f, g_f, closed_f)
        b_min = top_f(open_b, g_b, closed_b)

        # Correct termination: stop once neither side's *combined* remaining
        # potential can beat the best complete path found so far.
        if best <= f_min + b_min:
            break

        # Best-first alternation: always expand whichever frontier has the
        # lower minimum f, not whichever side has closed fewer nodes.
        expand_forward = f_min <= b_min

        if expand_forward:
            _, g_pop, _, current = heapq.heappop(open_f)
            if current in closed_f or g_pop > g_f.get(current, math.inf):
                continue
            closed_f.add(current)
            nodes_expanded += 1
            meet_cost(current)

            for nbr, edge_data in G.adj[current].items():
                if nbr in closed_f:
                    continue
                tg = g_f[current] + edge_cost(edge_data)
                if tg < g_f.get(nbr, math.inf):
                    g_f[nbr] = tg
                    came_f[nbr] = current
                    counter += 1
                    f = tg + heuristic(G, nbr, dest_lat, dest_lon, weight)
                    heapq.heappush(open_f, (f, tg, counter, nbr))
                    meet_cost(nbr)
        else:
            _, g_pop, _, current = heapq.heappop(open_b)
            if current in closed_b or g_pop > g_b.get(current, math.inf):
                continue
            closed_b.add(current)
            nodes_expanded += 1
            meet_cost(current)

            for nbr, edge_data in G.pred[current].items():
                if nbr in closed_b:
                    continue
                tg = g_b[current] + edge_cost(edge_data)
                if tg < g_b.get(nbr, math.inf):
                    g_b[nbr] = tg
                    came_b[nbr] = current
                    counter += 1
                    f = tg + heuristic(G, nbr, orig_lat, orig_lon, weight)
                    heapq.heappush(open_b, (f, tg, counter, nbr))
                    meet_cost(nbr)

    if meet is None:
        return SolverResult([], nodes_expanded, math.inf, "Bidirectional A*")

    route = []
    node = meet
    while True:
        route.append(node)
        if node == orig_node:
            break
        node = came_f[node]
    route.reverse()
    node = meet
    while node != dest_node:
        node = came_b[node]
        route.append(node)

    return SolverResult(route, nodes_expanded, best, "Bidirectional A*")