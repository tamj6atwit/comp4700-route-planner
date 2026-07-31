"""
Utility file for all solvers to use.
"""

from dataclasses import dataclass, field


# Upper bound on any edge's speed in the graph (km/h), used to convert a
# straight-line distance heuristic into a lower-bound *time* estimate when
# weight="travel_time". Must stay >= the fastest road present so the
# heuristic never overestimates true travel time (keeps A* admissible).
MAX_SPEED_KPH = 130.0
MAX_SPEED_MPS = MAX_SPEED_KPH / 3.6


@dataclass
class SolverResult:
    route: list           # list of node
    nodes_expanded: int   # number of nodes visited
    cost: float           # total path cost in the given `weight` units
    name: str = ""        # solver name, e.g. "A*"


def path_cost(G, route, weight="length"):
    """Utility: sum edge weights along a route. Handles MultiDiGraph parallel edges
    by taking the minimum-weight edge between consecutive nodes."""
    if not route or len(route) < 2:
        return 0.0
    total = 0.0
    for u, v in zip(route[:-1], route[1:]):
        edge_data = G.get_edge_data(u, v)
        total += min(data.get(weight, 1) for data in edge_data.values())
    return total
