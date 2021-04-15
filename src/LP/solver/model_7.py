"""
Trying a different approach - adding termination criteria
28 - 03 - 2021
"""


import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time
import pandas as pd
import csv
import sys
import math



# Custom termination criterion: Quit optimization
# - after  5s if a high quality (4% gap) solution has been found, or
# - after 20min if a feasible solution has been found.

def mycallback(model, where):
    if where == GRB.Callback.MIP:
        time = model.cbGet(GRB.Callback.RUNTIME)
        best = model.cbGet(GRB.Callback.MIP_OBJBST)
        bound = model.cbGet(GRB.Callback.MIP_OBJBND)

        if best < GRB.INFINITY:
            # We have a feasible solution
            if time > 5 and abs(bound - best) < 0.04 * abs(bound):
                model.terminate()

            if time > 1200:
                model.terminate()

# custom optimize() function that uses callback

def myopt(model):
    model.optimize(mycallback)

def create_model(hubs, clients, cost_matrix_1, cost_matrix_2):
    """
    Model to assign robots to clients respecting:
        1. Robot maximum capacity
        2. Robot maximum distance
        3. Client Time Windows
    :param hubs:
    :param clients:
    :param cost_matrix_1:
    :param cost_matrix_2:
    :return: robot assignment and demand for each hub
    """
    ### Start Time:
    start_time = time.time()

    ### Creating model:
    model = gp.Model("RAP")

    #####################################################
    # 1. sets & parameters (parameters should be already defined in __init__)
    #####################################################
    # a. define maximum number of robots:
    n_c = len(clients)  # number of customers
    R_max = len(clients) - 10 #TODO: The number of solutions (for Instance 1 - robots needed: 5)
    R = list(range(R_max))  # list of robots
    C = [c.name for c in clients]  # list of clients
    D_c = [c.demand for c in clients]  # list of demands
    Loc_c = [c.loc for c in clients]  # list of locations
    Loc_c_dict = {c.name: c.loc for c in clients}  # dict of locations - will not be used most probably
    L = list(set([l[0] for l in cost_matrix_2.keys()]))  # L = m+n
    H = list(range(len(hubs)))  # list of hubs (hubs 0,1,2)
    R_cap = 50  # maximum robot capacity
    R_dist = 200
    # fixed costs
    c_hub = 50
    c_robot = 10

    #####################################################
    # 2. decision variables
    #####################################################
    # hub open/not-open
    o = model.addVars(H, vtype=GRB.BINARY, name="o")
    # robot-client-hub assignment
    x = model.addVars(C, R, H, vtype=GRB.BINARY, name="x")
    # robot - hub assignment
    rob = model.addVars(R, H, vtype=GRB.BINARY, name="rob")
    # edges assignment to robots
    y = model.addVars(L, L, R, vtype=GRB.BINARY, name="y")

    #####################################################
    # 3. constraints
    #####################################################

    # 1. A robot must be assigned to a client
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R for h in H) == 1 for c in C), name="client_must_be_served")
    # 2. if robot serves a client, the corresponding hub must be open
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R) <= o[h] for c in C for h in H),
                     name="no_hub_no_robot")
    # 3. if robot serves a client, the corresponding rob decision variable must be 1
    model.addConstrs((rob[r, h] <= o[h] for r in R for h in H), name="no_hub_no_robot_2")

    # 4. At most one robot can be assigned to a job
    model.addConstrs((x.sum(c, '*', h) <= 1 for c in C for h in H), name="one_robot")

    # 5. Robot maximum capacity constraint
    model.addConstrs((gp.quicksum(D_c[int(c)] * x[c, r, h] for c in C for h in H) <= R_cap * rob.sum(r, '*') for r in R),
                     name="robot_cap")
    # 6. Distance covered by every robot should be less than maximum allowed distance
    model.addConstrs((gp.quicksum(cost_matrix_2[i, j] * y[i, j, r] for i in L for j in L if i != j)
                      <= R_dist for r in R), name="robot_dist")
    # Robot tour constraints
    # 7. a.	For each robot and client, we need to ensure that the
    # robot travels to another location after serving the client.
    model.addConstrs((gp.quicksum(y[int(j), int(i), r] for j in L if int(j) != int(i))
                      == gp.quicksum(x[i, r, h] for h in H) for r in R for i in C),
                        name="Tour1")
    model.addConstrs((gp.quicksum(y[int(i), int(j), r] for j in L if int(j) != int(i)) == gp.quicksum(x[i, r, h] for h in H) for r in R for i in C),
                 name="Tour2")

    # 7. b. Ensuring Tours in "y" are respecting the same hub the robot is serving from
    model.addConstrs((gp.quicksum(y[int(c), n_c + h, r] for c in C) == rob[r, h] for r in R for h in H),
                 name="sameHub1_y")
    model.addConstrs((gp.quicksum(y[n_c + h, int(c), r] for c in C) == rob[r, h] for r in R for h in H),
     name="sameHub2_y")

    # 7. c. Additional Tour Constraints (not needed):
    model.addConstrs((y[int(j), int(i), r] + y[int(i), int(j), r] <= 1 for r in R
                      for i in L for j in L if int(j) != int(i)), name="Tour3")

    # 8. Ensuring robot assignment in "x" are respecting the same hub the robot is serving from
    model.addConstrs((gp.quicksum(x[c, r, h] for c in C) <= (n_c + 1) * (rob[r, h]) for r in R for h in H),
                     name="sameHub_x")

    # 9. Robot serves from one hub only
    model.addConstrs((gp.quicksum(rob[r, h] for h in H) <= 1 for r in R), name="one_hub_per_robot")

    ########################################################
    # Subtour Elimination
    ########################################################
    # Miller-Tucker-Zemlin formulation (Bektas version):
    # How to define p?? The maximum number of clients a robot can visit, or the maximum number of hub a truck can visit

    p1 =  n_c
    u1 = model.addVars(L, vtype=GRB.INTEGER, ub=p1 + 1, lb=1, name="u")

    # 10. respecting the tour order
    model.addConstrs(((u1[i] - u1[j] + p1 * (gp.quicksum(y[i, j, r] for r in R))) <= p1 - 1
                      for i in range(0, p1) for j in range(0, p1) if i != j),
                     name="respect_order_clients")  # DH should be replaced by the sum of the open hubs

    #########################################################
    # 2nd part - Truck assignment
    #########################################################
    # defining the number of trucks by calculating the total demand
    V_cap = 300  # Truck Max Capacity

    total_demand = math.fsum(c.demand for c in clients)
    t = math.ceil((total_demand / V_cap))
    V = list(range(t+1))  # list of trucks - adding an additional truck for safety reasons

    c_truck = 50 # Truck Fixed Cost
    c_truck_distance = 5 # Truck Variable Cost

    DH = list(range(len(hubs) + 1)) # List of Depot and Hubs
    z = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="z")

    # Truck hub assignment
    w = model.addVars(V, vtype=GRB.BINARY, name="w")



    # 11. all the trucks must return to the depot station
    model.addConstrs((gp.quicksum(z[v, len(hubs), h] for h in DH[:-1]) == w[v] for v in V), name="trucks1")
    # 12. all trucks must depart from the depart station
    model.addConstrs((gp.quicksum(z[v, h, len(hubs)] for h in DH[:-1]) == w[v] for v in V), name="trucks2")
    # 13. sum of all trucks going to the same hub is 1 if hub is open
    model.addConstrs((gp.quicksum(z[v, h1, h2] for v in V for h1 in DH if h1 != h2)
                      == o[h2] for h2 in DH[:-1]), name="if_truck_then_hub")
    # 14. mirroring Constraint 11
    model.addConstrs((gp.quicksum(z[v, h1, h2] for v in V for h1 in DH if h1 != h2)
                      == gp.quicksum(z[v, h2, h1] for v in V for h1 in DH if h1 != h2)
                      for h2 in DH), name="trucks3")

    # Truck tour constraints (same as 13, 14)
    model.addConstrs((gp.quicksum(z[v, j, h] for j in DH for v in V if j != h)
                      == o[h] for h in H),
                     name="Tour1_truck")
    model.addConstrs((gp.quicksum(z[v, h, j] for j in DH for v in V if j != h)
                      == o[h] for h in H),
                     name="Tour2_truck")

    # 16. Demand per Hub
    D_h = model.addVars(H, ub=total_demand,name="D_h")

    # 17. Demand per hub
    model.addConstrs((D_h[h] == gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C for r in R) for h in H), name="total_demand_per_hub")
    # 18. Maximum Truck Capacity
    model.addConstrs((gp.quicksum(D_h[h]*z[v, i, h] for i in DH for h in H) <= V_cap*w[v] for v in V),
                        name="truck_capacity")

    # 19. Miller-Tucker-Zemlin formulation (Bektas 2006 version):
    # 1 <= u[.] <= p+1
    p = len(H) #len(DH) #n_c
    u2 = model.addVars(DH, vtype=GRB.INTEGER, ub=p + 1, lb=1, name="u2")

    # respecting the tour order
    model.addConstrs(((u2[h1] - u2[h2] + p*(gp.quicksum(z[v, h1, h2] for v in V))) <= p - 1
                      for h1 in range(0, p) for h2 in range(0, p) if h1 != h2),
                        name="respect_order")  # DH should be replaced by the sum of the open hubs

    #####################################################
    # 4. Objective function
    #####################################################

    # 1. robot distance cost
    cost_robot_var = gp.quicksum(cost_matrix_2[c1, c2] * y[c1, c2, r]
                                 for c1 in L for c2 in L if c1 != c2 for r in R)

    # 2. robot fixed cost
    cost_robot_fixed = gp.quicksum(c_robot * rob[r, h] for r in R for h in H)

    # 3. hub fixed cost
    cost_hub_fixed = gp.quicksum(c_hub * o[h] for h in H)

    # 4. truck distance cost
    cost_truck_var = gp.quicksum(cost_matrix_1[dh1, dh2] * c_truck_distance * z[v, dh1, dh2]
                                 for v in V for dh1 in DH for dh2 in DH if dh1 != dh2)

    # 5. truck fixed cost
    cost_truck_fixed = c_truck*gp.quicksum(w[v] for v in V)   # this will change in v2



    model.setObjective(cost_robot_var + cost_robot_fixed
                       + cost_hub_fixed + cost_truck_fixed
                       + cost_truck_var, GRB.MINIMIZE)

    #####################################################
    # 5. Saving LP model (remember to change versions)
    #####################################################
    model.write("../output/lp_model/RAP_TRP_model6_v2.lp")

    #### Optimize!
    #model.optimize()
    myopt(model)

    #####################################################
    # 6. analyzing solutions
    #####################################################

    print('Optimal solution found with total cost: %g' % model.objVal)

    for v in model.getVars():
        if v.x >= 0.5:
            print('%s %g' % (v.varName, v.x))

    ### Printing execution Time:
    print("Executed in %s Minutes" % ((time.time() - start_time)/60))

    # Robots:
    print("Analyzing Solutions...")
    print("1. Robots:")
    robots = {r: [] for r in R}
    active_robots = []
    for r in R:
        robot = ""
        hub = ""
        clients_served = []
        clients_served_real = []
        for h in H:
            for c in C:
                if x[c, r, h].X > 0.5:
                    robot = str(r+1)
                    hub = str(h+1)
                    clients_served.append(int(c)+1)
                    clients_served_real.append(int(c))
        if robot:
            print("Robot {}, will be serving from hub {}.".format(robot,hub))
            print("The robot will serve the following clients: {}".format(clients_served))
            active_robots.append(r)
        if hub:
            clients_served_real.append(n_c+int(hub)-1)
        robots[r] = clients_served_real

    # Robot tours:
    print("2. Robot tours:")
    tours = {r : [] for r in R}
    for r in R:
        links = list()
        for i in L:
            for j in L:
                if y[i, j, r].X > 0.5:
                    links.append((i,j))
        tours[r] = links
    print("Links visited by each robot: ")
    print(tours)

    #####################################################
    # 7. printing tours
    #####################################################

    import matplotlib.pyplot as plt
    import networkx as nx

    G = nx.DiGraph()
    list_nodes = list(range(1, len(L)))  # list(range(1, len(L) + 1))
    G.add_nodes_from(list_nodes)

    nodes_clients = {int(c.name): c.loc for c in clients}
    nodes_hubs = {n_c + h : hubs[h] for h in H}
    node_pos = {**nodes_clients, **nodes_hubs}

    # Create a list of nodes in shortest path
    for r in active_robots:
        # Create a list of edges in shortest path
        red_edges = [(i, j) for i in L for j in L if y[i, j, r].x > 0.5]
        for i in L:
            for j in L:
                if y[i, j, r].x > 0.5:
                    G.add_edge(i, j)
        # If the node is in the shortest path, set it to red, else set it to white color
        node_col = ['white' if not node in robots[r] else 'red' for node in G.nodes()]
        # If the edge is in the shortest path set it to red, else set it to white color
        edge_col = ['black' if not edge in red_edges else 'red' for edge in G.edges()]
        # Draw the nodes
        nx.draw_networkx(G, node_pos, node_color=node_col, node_size=450)
        # Draw the node labels
        # nx.draw_networkx_labels(G1, node_pos,node_color= node_col)
        # Draw the edges
        nx.draw_networkx_edges(G, node_pos, edge_color=edge_col)
        # Draw the edge labels
        #cost_matrix_2_to_int = cost_matrix_2
        #for sub in cost_matrix_2_to_int:
        #    cost_matrix_2_to_int[sub] = int(cost_matrix_2_to_int[sub])
        #nx.draw_networkx_edge_labels(G, node_pos, edge_color=edge_col, edge_labels=cost_matrix_2_to_int)
        # Remove the axis
        plt.axis('off')
        # TODO: Add description of the plot

        # Remove edges
        G.remove_edges_from(list(G.edges()))

        # Show the plot
        # plt.show()

        # Save the plot
        #plt.savefig("../output/plots/scenario1_satellite_double/Model_5_tour-robot_Ca2-3-15_{}.png".format(r + 1))
        #plt.savefig("../output/plots/scenario2_robot_capacity/Model_5_tour-robot_Ca2-3-15_{}.png".format(r + 1))
        #plt.savefig("../output/plots/scenario3_satellite_cost_robot_distance/Model_5_tour-robot_Ca2-3-15_{}.png".format(r + 1))
        #plt.savefig("../output/plots/normal_scenario/Model_5_tour-robot_Ca2-3-15_{}.png".format(r + 1))
        plt.savefig("../output/plots/Instances/Cd1-3,15/Model_7_tour_robot_{}.png".format(r + 1))
        #plt.savefig("../output/plots/Model_7_tour_robot_{}.png".format(r + 1))
        plt.clf()


