"""
Ayman Mahmoud - March 2021

Solution analyzer of the optimized model
"""

# Reads instances from the solver and creates plots
# you can check the example that used networkx. although it is not very helpful

import matplotlib.pyplot as plt
import networkx as nx

def print_solution_x(x):
    pass


def print_solution_y(y):
    pass


def print_solution_r(y, x, node_pos, L, R):

    G = nx.DiGraph()
    list_nodes = list(range(1, L + 1))
    G.add_nodes_from(list_nodes)

    for r in R:
        for i, j in enumerate(y) :
            G.add_edge(i, j)

    # Adding the position attribute to each node
    # node_pos = {1: (0, 0), 2: (2, 2), 3: (2, -2), 4: (5, 2), 5: (5, -2), 6: (7, 0)}

    # Create a list of edges in shortest path
    red_edges = [(i, j) for i, j in y if x[i, j].x > 0]

    # Create a list of nodes in shortest path
    for r in R:
        # If the node is in the shortest path, set it to red, else set it to white color
        node_col = ['white' if not node in x[r] else 'red' for node in G.nodes()]
        # If the edge is in the shortest path set it to red, else set it to white color
        edge_col = ['black' if not edge in red_edges else 'red' for edge in G.edges()]
        # Draw the nodes
        nx.draw_networkx(G, node_pos, node_color=node_col, node_size=450)
        # Draw the node labels
        # nx.draw_networkx_labels(G1, node_pos,node_color= node_col)
        # Draw the edges
        nx.draw_networkx_edges(G, node_pos, edge_color=edge_col)
        # Draw the edge labels
        nx.draw_networkx_edge_labels(G, node_pos, edge_color=edge_col, edge_labels=cost)
        # Remove the axis
        plt.axis('off')
        # TODO: Add description of the plot
        # TODO: Save the plot

        # Show the plot
        plt.show()