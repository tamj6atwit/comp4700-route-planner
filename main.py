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
    G = ox.graph_from_point(
        (40.7128, -74.0060), # Boston coordinates
        dist=1500,   # change for bigger or smaller area
        network_type="drive",
    )

    fig, ax = ox.plot_graph(
        G,
        node_size=8,
        node_color="red",
        edge_linewidth=0.5,
        bgcolor="white",
        show=True,
    )


if __name__ == "__main__":
    main()