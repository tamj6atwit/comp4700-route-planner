import osmnx as ox
import networkx as nx
import astar
import dijkstra
import bidirectional_astar
import bfs

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

def main():
    """
    Loads a graph from a point in the world
    Params: point (latitude, longitude), distance(meters), network_type(drive, walk, bike)
    """
    origin = (42.3551, -71.0656)       # near Boston Common
    destination = (42.3656, -71.0540)  # near North End / Haymarket

    G = ox.graph_from_point(
        (42.3601, -71.0589),  # Boston coordinates
        dist=1500,   # change for bigger or smaller area
        network_type="drive",
    )

    # nearest_nodes expects (longitude, latitude)
    orig_node = ox.nearest_nodes(G, origin[1], origin[0])
    dest_node = ox.nearest_nodes(G, destination[1], destination[0])

    route = ox.shortest_path(G, orig_node, dest_node, weight="length")

    fig, ax = ox.plot_graph_route(
        G,
        route,
        route_color="blue",
        route_linewidth=4,
        node_size=8,
        bgcolor="white",
        show=True,
    )


if __name__ == "__main__":
    main()