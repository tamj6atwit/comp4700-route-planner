import time

import osmnx as ox

import astar
import dijkstra
import bidirectional_astar

'''
Coordinates for major cities in USA
Boston: (42.3601, -71.0589)
New York: (40.7128, -74.0060)
Chicago: (41.8781, -87.6298)
Los Angeles: (34.0522, -118.2437)
Houston: (29.7604, -95.3698)
Miami: (25.7617, -80.1918)
Seattle: (47.6062, -122.3321)
San Francisco: (37.7749, -122.4194)
Washington DC: (38.8951, -77.0364)
'''

SOLVERS = {
    "astar": astar,
    "dijkstra": dijkstra,
    "bidirectional_astar": bidirectional_astar,
}


def load_graph(center_point, dist=1500, network_type="drive"):
    """
    Loads a graph from a point in the world
    Params: point (latitude, longitude), distance(meters), network_type(drive, walk, bike)

    Also annotates every edge with `speed_kph` (inferred from OSM `maxspeed`,
    falling back to highway-type defaults where missing) and `travel_time`
    (length / speed). This lets solvers route by `weight="travel_time"`
    instead of raw `weight="length"`, so a longer-but-faster highway can be
    preferred over a shorter-but-slower side street.
    """
    G = ox.graph_from_point(center_point, dist=dist, network_type=network_type)
    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)
    return G


def run_solver(solver_name, G, orig_node, dest_node, weight="length"):
    module = SOLVERS[solver_name]
    start = time.perf_counter()
    result = module.find_path(G, orig_node, dest_node, weight=weight)
    elapsed = time.perf_counter() - start

    print(f"[{solver_name}] route length: {len(result.route)} nodes | "
          f"cost: {result.cost:.1f} {weight} | "
          f"nodes expanded: {result.nodes_expanded} | "
          f"time: {elapsed * 1000:.2f} ms")
    return result, elapsed


def main():
    # Short pair (~2 km)
    origin_1 = (42.3502, -71.0750)       # Trinity Church
    destination_1 = (42.3656, -71.0540)  # near North End / Haymarket

    # Medium pair (~9 km)
    origin_2 = (42.3736, -71.1190)       # Harvard Square, Cambridge
    destination_2 = (42.3412, -71.0330)  # Seaport / Fort Point, South Boston

    # Long pair (~28 km straight-line)
    # origin_3 = (42.4230, -71.2066)       # Waltham / WNW edge
    # destination_3 = (42.2971, -70.9115)  # Quincy / ESE edge

    # Very long pair (~67 km straight-line) (Takes a while to run, commented out to skip)
    # origin_4 = (42.3601, -71.0589)       # Boston Common / downtown
    # destination_4 = (41.8236, -71.4222)  # Providence, RI (downtown)

    midpoint_1 = (
        (origin_1[0] + destination_1[0]) / 2,
        (origin_1[1] + destination_1[1]) / 2,
    )
    #midpoint_2 = (
    #    (origin_2[0] + destination_2[0]) / 2,
    #    (origin_2[1] + destination_2[1]) / 2,
    #)
    # midpoint_3 = (
    #     (origin_3[0] + destination_3[0]) / 2,
    #     (origin_3[1] + destination_3[1]) / 2,
    # )
    # midpoint_4 = (
    #     (origin_4[0] + destination_4[0]) / 2,
    #     (origin_4[1] + destination_4[1]) / 2,
    # )

    # Load the graph (dist sized to the pair so the map stays tight)
    G1 = load_graph(midpoint_1, dist=1000, network_type="drive")
    #G2 = load_graph(midpoint_2, dist=5000, network_type="drive")
    # G3 = load_graph(midpoint_3, dist=5000, network_type="drive")
    # G4 = load_graph(midpoint_4, dist=5000, network_type="drive")

    pairs = [
        ("Pair 1: Trinity → North End", origin_1, destination_1, G1),
        #("Pair 2: Harvard → Seaport", origin_2, destination_2, G2),
        # ("Pair 3: Waltham → Quincy edge", origin_3, destination_3, G3),
        # ("Pair 4: Boston → Providence", origin_4, destination_4, G4),
    ]

    for label, origin, destination, G in pairs:
        print(f"\n{'=' * 60}")
        print(f"{label}")
        print(f"{'=' * 60}")

        # nearest_nodes expects (longitude, latitude)
        orig_node = ox.nearest_nodes(G, origin[1], origin[0])
        dest_node = ox.nearest_nodes(G, destination[1], destination[0])

        # Run every solver that's implemented so far, once per weight, so we can
        # compare shortest-distance routing vs shortest-time routing (the latter
        # is what lets a solver prefer a longer-but-faster highway).
        results = {}
        for weight in ("length", "travel_time"):
            print(f"\n--- Weight: {weight} ---")
            for name in SOLVERS:
                try:
                    result, elapsed = run_solver(name, G, orig_node, dest_node, weight=weight)
                    results[(name, weight)] = (result, elapsed)
                except NotImplementedError:
                    print(f"[{name}] not implemented yet, skipping")

        # Plot A*'s distance-optimal route vs its time-optimal route side by side,
        # if both are available.
        routes, colors, labels = [], [], []
        if ("astar", "length") in results:
            routes.append(results[("astar", "length")][0].route)
            colors.append("blue")
            labels.append("shortest distance")
        if ("astar", "travel_time") in results:
            routes.append(results[("astar", "travel_time")][0].route)
            colors.append("red")
            labels.append("shortest time")

        if routes:
            ox.plot_graph_routes(
                G,
                routes,
                route_colors=colors,
                route_linewidth=4,
                node_size=8,
                bgcolor="white",
                show=True,
            ) if len(routes) > 1 else ox.plot_graph_route(
                G,
                routes[0],
                route_color=colors[0],
                route_linewidth=4,
                node_size=8,
                bgcolor="white",
                show=True,
            )
            print(f"\nPlotted routes for {label}:",
                  ", ".join(f"{c}={l}" for c, l in zip(colors, labels)))


if __name__ == "__main__":
    main()
