"""
Ayman Mahmoud - March 2021

All functions related to the gurobi model
"""
# Imports
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import pandas as pd
import csv
import sys
import math

def create_model_1(depot, hubs, clients,
                   cost_matrix_1, cost_matrix_2):
    """
    Model 1 = A simple 2E-CVRP with no time window constraints

    :param robots: an object with location and number
    :param depot: location of the depot station, name of the depot
    :param hubs: each hub has a location
    :param clients: clients have names,
    :param cost_matrix_1: a dict that uses the keys of locations to give you the distance (Depot - Hubs)
    :param cost_matrix_2: a dict that uses the keys of locations to give you the distance (Hubs-clients, Hubs-clients)
    (ex. cost_matrix_1[h.name,d.name] = distance between hub i and depot j)
    :return:  Gurobi model with the related constraints
    """
    ### Creating model:
    model = gp.Model("2E-VRP")

    #####################################################
    # 1. sets & parameters (parameters should be already defined in __init__)
    #####################################################
            # a. define maximum number of robots:
            # TODO: move this to create an object list of robots
    n_c = len(clients) # number of customers
    R_max = len(clients)
    R = list(range(R_max)) # list of robots
    Dep = list(range(len(depot)))  # list of depots
    C = [c.name for c in clients] # list of clients
    D_c = [c.demand for c in clients] # list of demands
    L = list(set([l[0] for l in cost_matrix_2.keys()])) # L = m+n
    # TODO: transform L to be a set of str.
    H = list(range(len(hubs))) # list of hubs (hubs 0,1,2)
    Loc_c = [c.loc for c in clients] # list of locations
    Loc_c_dict = {c.name : c.loc for c in clients} # dict of locations - will not be used most probably
    R_cap = 50 # maximum robot capacity
    R_dist = 200
    V_cap = 300
    # defining the number of trucks by calculating the total demand
    total_demand = math.fsum(c.demand for c in clients)
    t = math.ceil((total_demand/V_cap))
    V = list(range(t)) # list of trucks
    # fixed costs
    c_hub = 50
    c_truck = 50
    c_robot = 10
    # variable cost
    c_truck_distance = 5

    #####################################################
    # 2. decision variables
    #####################################################
    # hub open/not-open
    o = model.addVars(H, vtype=GRB.BINARY, name="o")
    # robot-client-hub assignment
    x = model.addVars(C, R, H, vtype=GRB.BINARY, name="x")
    # robot assignment
    robot = model.addVars(R, vtype=GRB.BINARY, name="robot")
    # van-hub assignment
    # TODO: compare with the TSP formulation
    # create a list of Depots + Hubs
    # DH = [0]+H # in our case it is only one depot, the model is extendable to include multiple (if we..)
    DH = list(range(len(hubs)+1))
    # TODO: remember in the distance matrix hubs are 0,1,2 and depots are 3
    # y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y")
    y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y")
    # TODO: set the variable to 0 when dh1 = dh2
    # edges assignment to robots
    z = model.addVars(L, L, R, vtype=GRB.BINARY, name="z")
    # a new decision variable to assign robots to hubs
    # w = model.addVars(H, R, vtype=GRB.BINARY, name="w")
    #####################################################
    # 3. constraints
    #####################################################
    # (2) - all client nodes are assigned to hubs
    # implemented in constraint (D)
    # (4) - if robot serves a client, the corresponding hub must be open
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R) <= o[h] for c in C for h in H),
                     name="no_hub_no_robot")
    # (8) - all the trucks must return to the depot station
    model.addConstrs((gp.quicksum(y[v, dh, len(H)] for dh in DH[:-1] if dh != len(H)) == t for v in V), name="trucks1")
    # (9) - all trucks must depart from the depart station
    model.addConstrs((gp.quicksum(y[v, len(H), dh] for dh in DH[:-1] if dh != len(H)) == t for v in V), name="trucks2")
    # (10) - number of trucks leaving = number of trucks returning
    # TODO: review indexing
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH[:-1] if dh1 != dh2)
                      == gp.quicksum(y[v, dh2, dh1] for v in V for dh1 in DH[:-1] if dh1 != dh2) for dh2 in DH[1:]),
                        name="trucks3")
    # (11) - sum of all trucks going to the same hub is 1 if hub is open
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH[:-1] if dh1 != dh2)
                      == o[dh2] for dh2 in DH[:-1]), name="if_truck_then_hub")  # TODO: test with putting cstr 11 before 4
    # (16) - maximum truck capacity - needs correction
    #model.addConstrs(((gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C for r in R for h in H))
     #                 <= V_cap*(gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH for dh2 in DH if dh1 != dh2))),
      #               name="truck_cap")
    # (17) - adding constraint 17 from the article for testing
    model.addConstr((gp.quicksum(o[h] for h in H) ==len(H)),name="constraint_17")
    # we shouldn't need constraint 17 if the variable o is activated by a constraint x which is already written


    # (A) - distance covered by every robot should be less than maximum allowed distance
    model.addConstrs((gp.quicksum(cost_matrix_2[c1, c2] * z[c1, c2, r] for c1 in L for c2 in L if c1 != c2)
                      <= R_dist*robot[r] for r in R),
                     name="robot_dist")
    # forming a tour contstraint (22) :
        # (B1) - ensure that the robot assigned to a client travels to another location after every client
    model.addConstrs((gp.quicksum(z[int(c1), int(c2), r] for c1 in C if c1 != c2) == gp.quicksum(x[c2, r, h] for h in H) for r in R for c2 in C),
                 name="tour1")
    # (B2) - same but from j --> i
    model.addConstrs((gp.quicksum(z[int(c2), int(c1), r] for c1 in C if c1 != c2) == gp.quicksum(x[c2, r, h] for h in H) for r in R for c2 in C),
                 name="tour2")
    # Another solution would be to add a very high cost to the same node.
    # maybe will need to add constraint to the depot.
    # (B3) - ensure that each robot forms a tour from and to the same hub
    # Same hub constraints:
    # TODO: compare between running with B3 and B4 and without
    # another solution is to assign robots to hub prior to optimization
    model.addConstrs((gp.quicksum(z[int(c), n_c + h, r] for c in C for h in H) == robot[r] for r in R),
                 name="sameHub1")
    # (B4) - same as (B3) but from j --> i
    model.addConstrs((gp.quicksum(z[n_c + h, int(c), r] for c in C for h in H) == robot[r] for r in R),
                 name="sameHub2")

    # (C) - respect maximum robot capacity
    model.addConstrs((gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C for h in H) <= R_cap*robot[r] for r in R),
                     name="robot_cap")
    # (D) - each client is served by a single robot from a single hub
    model.addConstrs((gp.quicksum(x[c, r, h] for h in H for r in R) == 1 for c in C), name="Just_one_hub")
    # (E) - same as (D) but summing over all clients and all hubs
    model.addConstrs((x.sum(c, '*', h) <= 1 for c in C for h in H), name="Just_one_robot")

    # (R) - decision variable r constraints - needs revision
    #model.addConstrs(robot[r] == gp.quicksum(z[int(c1), int(c2), r] for c1 in C for c2 in C if c1 != c2) for r in R)
    # model.addConstrs((gp.quicksum(x[c, r, h] for c in C for h in H) <= robot[r] for r in R),
      #               name="no_robot_no_robot")

    #####################################################
    # 4. Objective function
    #####################################################

    # 1. truck distance cost
    cost_truck_var = gp.quicksum(cost_matrix_1[dh1,dh2]*c_truck_distance*y[v, dh1, dh2]
                                 for v in V for dh1 in DH for dh2 in DH if dh1 != dh2)

    # 2. truck fixed cost
    cost_truck_fixed = c_truck*t # this will change in v2

    # 3. robot distance cost
    cost_robot_var = gp.quicksum(cost_matrix_2[c1, c2] * z[c1, c2, r]
                                   for c1 in L for c2 in L if c1 != c2 for r in R)

    # 4. robot fixed cost
    cost_robot_fixed = gp.quicksum(c_robot*robot[r] for r in R)

    # 5. hub fixed cost
    cost_hub_fixed = gp.quicksum(c_hub*o[h] for h in H) #H[:-1]

    model.setObjective(cost_truck_var + cost_truck_fixed +
                       cost_robot_var + cost_robot_fixed
                       + cost_hub_fixed, GRB.MINIMIZE)

    #####################################################
    # 5. Saving LP model (remember to change versions)
    #####################################################
    model.write("../output/lp_model/2EVRP_v1.lp")

    #### Optimize!
    model.optimize()

    status = model.Status
    if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
        print("Model is either infeasible or unbounded.")
        sys.exit(0)
    elif status != GRB.OPTIMAL:
        print("Optimization terminated with status {}".format(status))
        sys.exit(0)


    #

    # TODO: move to solution printer
    # solution_printer.print_solution_x(x)
    # solution_printer.print_solution_y(y)
    # solution_printer.print_solution_r(r)


def create_model_2(depot, hubs, clients,
                   cost_matrix_1, cost_matrix_2):
    """
    Model 1 = A simple 2E-CVRP with no time window constraints

    :param robots: an object with location and number
    :param depot: location of the depot station, name of the depot
    :param hubs: each hub has a location
    :param clients: clients have names,
    :param cost_matrix_1: a dict that uses the keys of locations to give you the distance (Depot - Hubs)
    :param cost_matrix_2: a dict that uses the keys of locations to give you the distance (Hubs-clients, Hubs-clients)
    (ex. cost_matrix_1[h.name,d.name] = distance between hub i and depot j)
    :return:  Gurobi model with the related constraints
    """
    ### Creating model:
    model = gp.Model("2E-VRP")

    #####################################################
    # 1. sets & parameters (parameters should be already defined in __init__)
    #####################################################
            # a. define maximum number of robots:
            # TODO: move this to create an object list of robots
    n_c = len(clients) # number of customers
    R_max = len(clients)
    R = list(range(R_max)) # list of robots
    Dep = list(range(len(depot)))  # list of depots
    C = [c.name for c in clients] # list of clients
    D_c = [c.demand for c in clients] # list of demands
    L = list(set([l[0] for l in cost_matrix_2.keys()])) # L = m+n
    # TODO: transform L to be a set of str.
    H = list(range(len(hubs))) # list of hubs (hubs 0,1,2)
    Loc_c = [c.loc for c in clients] # list of locations
    Loc_c_dict = {c.name : c.loc for c in clients} # dict of locations - will not be used most probably
    R_cap = 50 # maximum robot capacity
    R_dist = 200
    V_cap = 300
    # defining the number of trucks by calculating the total demand
    total_demand = math.fsum(c.demand for c in clients)
    t = math.ceil((total_demand/V_cap))
    V = list(range(t)) # list of trucks
    # fixed costs
    c_hub = 50
    c_truck = 50
    c_robot = 10
    # variable cost
    c_truck_distance = 5

    #####################################################
    # 2. decision variables
    #####################################################
    # hub open/not-open
    o = model.addVars(H, vtype=GRB.BINARY, name="o")
    # robot-client-hub assignment
    x = model.addVars(C, R, H, vtype=GRB.BINARY, name="x")
    # robot assignment
    # robot = model.addVars(R, vtype=GRB.BINARY, name="robot")
    # van-hub assignment
    # TODO: compare with the TSP formulation
    # create a list of Depots + Hubs
    # DH = [0]+H # in our case it is only one depot, the model is extendable to include multiple (if we..)
    DH = list(range(len(hubs)+1))
    # TODO: remember in the distance matrix hubs are 0,1,2 and depots are 3
    # y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y")
    y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y")
    # TODO: set the variable to 0 when dh1 = dh2
    # edges assignment to robots
    z = model.addVars(L, L, R, vtype=GRB.BINARY, name="z")
    # a new decision variable to assign robots to hubs
    w = model.addVars(R, H, vtype=GRB.BINARY, name="w")
    #####################################################
    # 3. constraints
    #####################################################
    # (2) - all client nodes are assigned to hubs
    # implemented in constraint (D)
    # (4) - if robot serves a client, the corresponding hub must be open
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R) <= o[h] for c in C for h in H),
                     name="no_hub_no_robot")
    # (8) - all the trucks must return to the depot station
    model.addConstrs((gp.quicksum(y[v, dh, len(H)] for dh in DH[:-1] if dh != len(H)) == t for v in V), name="trucks1")
    # (9) - all trucks must depart from the depart station
    model.addConstrs((gp.quicksum(y[v, len(H), dh] for dh in DH[:-1] if dh != len(H)) == t for v in V), name="trucks2")
    # (10) - number of trucks leaving = number of trucks returning
    # TODO: review indexing
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH[:-1] if dh1 != dh2)
                      == gp.quicksum(y[v, dh2, dh1] for v in V for dh1 in DH[:-1] if dh1 != dh2) for dh2 in DH[1:]),
                        name="trucks3")
    # (11) - sum of all trucks going to the same hub is 1 if hub is open
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH[:-1] if dh1 != dh2)
                      == o[dh2] for dh2 in DH[:-1]), name="if_truck_then_hub")  # TODO: test with putting cstr 11 before 4
    # (16) - maximum truck capacity - needs correction
    #model.addConstrs(((gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C for r in R for h in H))
     #                 <= V_cap*(gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH for dh2 in DH if dh1 != dh2))),
      #               name="truck_cap")

    # (17) - adding constraint 17 from the article for testing
    model.addConstr((gp.quicksum(o[h] for h in H) ==len(H)),name="constraint_17")
    # we shouldn't need constraint 17 if the variable o is activated by a constraint x which is already written


    # (A) - distance covered by every robot should be less than maximum allowed distance
    model.addConstrs((gp.quicksum(cost_matrix_2[c1, c2] * z[c1, c2, r] for c1 in L for c2 in L if c1 != c2)
                      <= R_dist for r in R), name="robot_dist")
    # forming a tour contstraint (22) :
        # (B1) - ensure that the robot assigned to a client travels to another location after every client
    model.addConstrs((gp.quicksum(z[int(c1), int(c2), r] for c1 in C if c1 != c2) == gp.quicksum(x[c2, r, h] for h in H) for r in R for c2 in C),
                 name="tour1")
    # (B2) - same but from j --> i
    model.addConstrs((gp.quicksum(z[int(c2), int(c1), r] for c1 in C if c1 != c2) == gp.quicksum(x[c2, r, h] for h in H) for r in R for c2 in C),
                 name="tour2")
    # Another solution would be to add a very high cost to the same node.
    # maybe will need to add constraint to the depot.
    # (B3) - ensure that each robot forms a tour from and to the same hub
    # Same hub constraints:
    # TODO: compare between running with B3 and B4 and without
    # another solution is to assign robots to hub prior to optimization
    model.addConstrs((gp.quicksum(z[int(c), n_c + h, r] for c in C) == w[r, h] for r in R for h in H),
                 name="sameHub1")
    # (B4) - same as (B3) but from j --> i
    model.addConstrs((gp.quicksum(z[n_c + h, int(c), r] for c in C) == w[r, h] for r in R for h in H),
                 name="sameHub2")

    # (C) - respect maximum robot capacity
    model.addConstrs((gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C) <= R_cap*w[r, h] for r in R for h in H),
                     name="robot_cap")
    # (D) - each client is served by a single robot from a single hub
    model.addConstrs((gp.quicksum(x[c, r, h] for h in H for r in R) == 1 for c in C), name="Just_one_hub")
    # (E) - same as (D) but summing over all clients and all hubs
    model.addConstrs((x.sum(c, '*', h) <= 1 for c in C for h in H), name="Just_one_robot")

    # (R) - decision variable r constraints - needs revision
    #model.addConstrs(robot[r] == gp.quicksum(z[int(c1), int(c2), r] for c1 in C for c2 in C if c1 != c2) for r in R)
    # model.addConstrs((gp.quicksum(x[c, r, h] for c in C for h in H) <= robot[r] for r in R),
      #               name="no_robot_no_robot")

    # TODO: check if we need constraints on w
    # each robot is assigned to one hub
    model.addConstrs((w.sum(r, '*') <= 1 for r in R), name="Just_one_hub_for_robot")

    # Additional constraints for "z"
    model.addConstrs((gp.quicksum(z[c1, c2, r] for r in R) <= 1 for c1 in L for c2 in L if c1 != c2), name="only_one_robot_z")

    #####################################################
    # 4. Objective function
    #####################################################

    # 1. truck distance cost
    cost_truck_var = gp.quicksum(cost_matrix_1[dh1,dh2]*c_truck_distance*y[v, dh1, dh2]
                                 for v in V for dh1 in DH for dh2 in DH if dh1 != dh2)

    # 2. truck fixed cost
    cost_truck_fixed = c_truck*t # this will change in v2

    # 3. robot distance cost
    cost_robot_var = gp.quicksum(cost_matrix_2[c1, c2] * z[c1, c2, r]
                                   for c1 in L for c2 in L if c1 != c2 for r in R)

    # 4. robot fixed cost
    cost_robot_fixed = gp.quicksum(c_robot*w[r, h] for r in R for h in H)

    # 5. hub fixed cost
    cost_hub_fixed = gp.quicksum(c_hub*o[h] for h in H)

    model.setObjective(cost_truck_var + cost_truck_fixed +
                       cost_robot_var + cost_robot_fixed
                       + cost_hub_fixed, GRB.MINIMIZE)

    #####################################################
    # 5. Saving LP model (remember to change versions)
    #####################################################
    model.write("../output/lp_model/2EVRP_v2.lp")

    #### Optimize!
    model.optimize()

    status = model.Status
    if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
        print("Model is either infeasible or unbounded.")
        sys.exit(0)
    elif status != GRB.OPTIMAL:
        print("Optimization terminated with status {}".format(status))
        sys.exit(0)

    #####################################################
    # 6. analyzing solutions
    #####################################################
    print('Optimal solution found with total cost: %g' % model.objVal)

    for v in model.getVars():
        print('%s %g' % (v.varName, v.x))

    # 1. analyzing hubs
    print("Analyzing hubs")

    # 2. analyzing robots
    print("Analyzing Robots")

    # 3. analyzing tours (echelon 1)
    print("Analyzing tours (depot - hubs)")

    # 4. analyzing tours (echelon 2)
    print("Analyzing tours (hubs - clients)")


    # TODO: move to solution printer
    # solution_printer.print_solution_x(x)
    # solution_printer.print_solution_y(y)
    # solution_printer.print_solution_r(r)


############################################################################

def create_model_3(depot, hubs, clients,
                   cost_matrix_1, cost_matrix_2):
    """
    Model 3 = A simple 2E-CVRP with no tour decision variable

    :param robots: an object with location and number
    :param depot: location of the depot station, name of the depot
    :param hubs: each hub has a location
    :param clients: clients have names,
    :param cost_matrix_1: a dict that uses the keys of locations to give you the distance (Depot - Hubs)
    :param cost_matrix_2: a dict that uses the keys of locations to give you the distance (Hubs-clients, Hubs-clients)
    (ex. cost_matrix_1[h.name,d.name] = distance between hub i and depot j)
    :return:  Gurobi model with the related constraints
    """
    ### Creating model:
    model = gp.Model("2E-VRP")

    #####################################################
    # 1. sets & parameters (parameters should be already defined in __init__)
    #####################################################
            # a. define maximum number of robots:
            # TODO: move this to create an object list of robots
    n_c = len(clients) # number of customers
    R_max = len(clients)
    R = list(range(R_max)) # list of robots
    Dep = list(range(len(depot)))  # list of depots
    C = [c.name for c in clients] # list of clients
    D_c = [c.demand for c in clients] # list of demands
    L = list(set([l[0] for l in cost_matrix_2.keys()])) # L = m+n
    # TODO: transform L to be a set of str.
    H = list(range(len(hubs))) # list of hubs (hubs 0,1,2)
    Loc_c = [c.loc for c in clients] # list of locations
    Loc_c_dict = {c.name : c.loc for c in clients} # dict of locations - will not be used most probably
    R_cap = 50 # maximum robot capacity
    R_dist = 200
    V_cap = 300
    # defining the number of trucks by calculating the total demand
    total_demand = math.fsum(c.demand for c in clients)
    t = math.ceil((total_demand/V_cap))
    V = list(range(t)) # list of trucks
    # fixed costs
    c_hub = 50
    c_truck = 50
    c_robot = 10
    # variable cost
    c_truck_distance = 5

    #####################################################
    # 2. decision variables
    #####################################################
    # hub open/not-open
    o = model.addVars(H, vtype=GRB.BINARY, name="o")
    # robot-client-hub assignment
    x = model.addVars(C, R, H, vtype=GRB.BINARY, name="x")
    # robot assignment
    # robot = model.addVars(R, vtype=GRB.BINARY, name="robot")
    # van-hub assignment
    # TODO: compare with the TSP formulation
    # create a list of Depots + Hubs
    # DH = [0]+H # in our case it is only one depot, the model is extendable to include multiple (if we..)
    DH = list(range(len(hubs)+1))
    # TODO: remember in the distance matrix hubs are 0,1,2 and depots are 3
    # y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y")
    y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y") # TODO: you need the subtour elimination constraint
    # TODO: set the variable to 0 when dh1 = dh2
    # edges assignment to robots
    # z = model.addVars(L, L, R, vtype=GRB.BINARY, name="z")
    # a new decision variable to assign robots to hubs
    w = model.addVars(R, H, vtype=GRB.BINARY, name="w")
    #####################################################
    # 3. constraints
    #####################################################
    # (2) - all client nodes are assigned to hubs
    # implemented in constraint (D)
    # (4) - if robot serves a client, the corresponding hub must be open
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R) <= o[h] for c in C for h in H),
                     name="no_hub_no_robot")
    # (8) - all the trucks must return to the depot station
    model.addConstrs((gp.quicksum(y[v, dh, len(H)] for dh in DH[:-1] if dh != len(H)) == t for v in V), name="trucks1")
    # (9) - all trucks must depart from the depart station
    model.addConstrs((gp.quicksum(y[v, len(H), dh] for dh in DH[:-1] if dh != len(H)) == t for v in V), name="trucks2")
    # (10) - number of trucks leaving = number of trucks returning
    # TODO: review indexing
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH[:-1] if dh1 != dh2)
                      == gp.quicksum(y[v, dh2, dh1] for v in V for dh1 in DH[:-1] if dh1 != dh2) for dh2 in DH[1:]),
                        name="trucks3")
    # (11) - sum of all trucks going to the same hub is 1 if hub is open
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH[:-1] if dh1 != dh2)
                      == o[dh2] for dh2 in DH[:-1]), name="if_truck_then_hub")  # TODO: test with putting cstr 11 before 4
    # (16) - maximum truck capacity - needs correction
    #model.addConstrs(((gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C for r in R for h in H))
     #                 <= V_cap*(gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH for dh2 in DH if dh1 != dh2))),
      #               name="truck_cap")

    # (17) - adding constraint 17 from the article for testing
    # model.addConstr((gp.quicksum(o[h] for h in H) == len(H)),name="constraint_17")
    # we shouldn't need constraint 17 if the variable o is activated by a constraint x which is already written


    # (A) - distance covered by every robot should be less than maximum allowed distance
    #model.addConstrs((gp.quicksum(cost_matrix_2[c1, c2] * z[c1, c2, r] for c1 in L for c2 in L if c1 != c2)
    #                  <= R_dist for r in R), name="robot_dist")
    # forming a tour contstraint (22) :
        # (B1) - ensure that the robot assigned to a client travels to another location after every client
    #model.addConstrs((gp.quicksum(z[int(c1), int(c2), r] for c1 in C if c1 != c2) == gp.quicksum(x[c2, r, h] for h in H) for r in R for c2 in C),
     #            name="tour1")
    # (B2) - same but from j --> i
    #model.addConstrs((gp.quicksum(z[int(c2), int(c1), r] for c1 in C if c1 != c2) == gp.quicksum(x[c2, r, h] for h in H) for r in R for c2 in C),
     #            name="tour2")
    # Another solution would be to add a very high cost to the same node.
    # maybe will need to add constraint to the depot.
    # (B3) - ensure that each robot forms a tour from and to the same hub
    # Same hub constraints:
    # TODO: compare between running with B3 and B4 and without
    # another solution is to assign robots to hub prior to optimization
    # model.addConstrs((gp.quicksum(z[int(c), n_c + h, r] for c in C) == w[r, h] for r in R for h in H),
      #           name="sameHub1")
    # (B4) - same as (B3) but from j --> i
    #model.addConstrs((gp.quicksum(z[n_c + h, int(c), r] for c in C) == w[r, h] for r in R for h in H),
     #            name="sameHub2")

    # (C) - respect maximum robot capacity
    model.addConstrs((gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C) <= R_cap*w[r, h] for r in R for h in H),
                     name="robot_cap")
    # (D) - each client is served by a single robot from a single hub
    model.addConstrs((gp.quicksum(x[c, r, h] for h in H for r in R) == 1 for c in C), name="Just_one_hub")
    # (E) - same as (D) but summing over all clients and all hubs
    model.addConstrs((x.sum(c, '*', h) <= 1 for c in C for h in H), name="Just_one_robot")
    # (D) is shadowed by (E)

    # (WO) - 25/03 - set o == 1 if w == 1
    model.addConstrs(gp.quicksum(w[r, h] for r in R) <= R_max*o[h] for h in H)

    # (R) - decision variable r constraints - needs revision
    #model.addConstrs(robot[r] == gp.quicksum(z[int(c1), int(c2), r] for c1 in C for c2 in C if c1 != c2) for r in R)
    # model.addConstrs((gp.quicksum(x[c, r, h] for c in C for h in H) <= robot[r] for r in R),
      #               name="no_robot_no_robot")

    # TODO: check if we need constraints on w
    # each robot is assigned to one hub
    model.addConstrs((w.sum(r, '*') <= 1 for r in R), name="Just_one_hub_for_robot")

    # Additional constraints for "z"
    # model.addConstrs((gp.quicksum(z[c1, c2, r] for r in R) <= 1 for c1 in L for c2 in L if c1 != c2), name="only_one_robot_z")

    #####################################################
    # 4. Objective function
    #####################################################

    # 1. truck distance cost
    cost_truck_var = gp.quicksum(cost_matrix_1[dh1,dh2]*c_truck_distance*y[v, dh1, dh2]
                                 for v in V for dh1 in DH for dh2 in DH if dh1 != dh2)

    # 2. truck fixed cost
    cost_truck_fixed = c_truck*t # this will change in v2

    # 3. robot distance cost
    #cost_robot_var = gp.quicksum(cost_matrix_2[c1, c2] * z[c1, c2, r]
     #                              for c1 in L for c2 in L if c1 != c2 for r in R)

    # 4. robot fixed cost
    cost_robot_fixed = gp.quicksum(c_robot*w[r, h] for r in R for h in H)

    # 5. hub fixed cost
    cost_hub_fixed = gp.quicksum(c_hub*o[h] for h in H)

    model.setObjective(cost_truck_var + cost_truck_fixed + cost_robot_fixed
                       + cost_hub_fixed, GRB.MINIMIZE)

    #####################################################
    # 5. Saving LP model (remember to change versions)
    #####################################################
    model.write("../output/lp_model/2EVRP_v2.lp")

    #### Optimize!
    model.optimize()

    status = model.Status
    if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
        print("Model is either infeasible or unbounded.")
        sys.exit(0)
    elif status != GRB.OPTIMAL:
        print("Optimization terminated with status {}".format(status))
        sys.exit(0)

    #####################################################
    # 6. analyzing solutions
    #####################################################
    print('Optimal solution found with total cost: %g' % model.objVal)

    for v in model.getVars():
        print('%s %g' % (v.varName, v.x))

    # 1. analyzing hubs
    print("Analyzing hubs")

    # 2. analyzing robots
    print("Analyzing Robots")

    # 3. analyzing tours (echelon 1)
    print("Analyzing tours (depot - hubs)")

    # 4. analyzing tours (echelon 2)
    print("Analyzing tours (hubs - clients)")


    # TODO: move to solution printer
    # solution_printer.print_solution_x(x)
    # solution_printer.print_solution_y(y)
    # solution_printer.print_solution_r(r)


#########################################################""

def create_model_4(depot, hubs, clients,
                   cost_matrix_1, cost_matrix_2):
    """
    Model 4 = A simple CVRP with no clients

    :param robots: an object with location and number
    :param depot: location of the depot station, name of the depot
    :param hubs: each hub has a location
    :param clients: clients have names,
    :param cost_matrix_1: a dict that uses the keys of locations to give you the distance (Depot - Hubs)
    :param cost_matrix_2: a dict that uses the keys of locations to give you the distance (Hubs-clients, Hubs-clients)
    (ex. cost_matrix_1[h.name,d.name] = distance between hub i and depot j)
    :return:  Gurobi model with the related constraints
    """
    ### Creating model:
    model = gp.Model("2E-VRP")

    #####################################################
    # 1. sets & parameters (parameters should be already defined in __init__)
    #####################################################
            # a. define maximum number of robots:
            # TODO: move this to create an object list of robots
    n_c = len(clients) # number of customers
    R_max = len(clients)
    R = list(range(R_max)) # list of robots
    # Dep = list(range(len(depot)))  # list of depots - this one is wrong
    C = [c.name for c in clients] # list of clients
    D_c = [c.demand for c in clients] # list of demands
    L = list(set([l[0] for l in cost_matrix_2.keys()])) # L = m+n
    # TODO: transform L to be a set of str.
    H = list(range(len(hubs)+1)) # list of hubs (hubs 0,1,2)
    Loc_c = [c.loc for c in clients] # list of locations
    Loc_c_dict = {c.name : c.loc for c in clients} # dict of locations - will not be used most probably
    R_cap = 50 # maximum robot capacity
    R_dist = 200
    V_cap = 300
    # defining the number of trucks by calculating the total demand
    total_demand = math.fsum(c.demand for c in clients)
    t = math.ceil((total_demand/V_cap))
    #V = list(range(t)) # list of trucks
    # fixed costs
    c_hub = 50
    c_truck = 50
    c_robot = 10
    # variable cost
    c_truck_distance = 5
    p = 4 # p will be replaced by the sum of


    #####################################################
    # 2. decision variables
    #####################################################
    # hub open/not-open
    o = model.addVars(H, vtype=GRB.BINARY, name="o")
    # DH = [0]+H # in our case it is only one depot, the model is extendable to include multiple (if we..)
    DH = list(range(len(hubs)+2))
    # TODO: remember in the distance matrix hubs are 0,1,2 and depots are 3
    y = model.addVars(DH, DH, vtype=GRB.BINARY, name="y") # TODO: you need the subtour elimination constraint
    # TODO: set the variable to 0 when dh1 = dh2
    # TODO: Check if we need the decision variable for the truck
    # 1 <= u[.] <= p+1
    u = model.addVars(DH, vtype=GRB.INTEGER, ub=p+1, lb=1, name="u") # u[V,DH]



    #####################################################
    # 3. constraints
    #####################################################

    # (8) - all the trucks must return to the depot station
    #model.addConstr((gp.quicksum(y[h, 0] for h in DH[1:]) == 1), name="trucks1")
    # (9) - all trucks must depart from the depart station
    #model.addConstr((gp.quicksum(y[0, h] for h in DH[1:]) == 1), name="trucks2")
    # (10) - number of trucks leaving = number of trucks returning
    model.addConstrs((gp.quicksum(y[h1, h2] for h1 in DH[1:] if h1 != h2)
                      == gp.quicksum(y[h2, h1] for h1 in DH[1:] if h1 != h2)
                      for h2 in DH[1:]), name="trucks3")
    # (11) - sum of all trucks going to the same hub is 1 if hub is open
    model.addConstrs((gp.quicksum(y[h1, h2] for h1 in DH[1:] if h1 != h2)
                      == o[h2-1] for h2 in DH[1:]), name="if_truck_then_hub")

    # (12) - setting the starting point from main Depot
    model.addConstr((u[0] == 1), name="start_from_depot")

    # (13) - order of satellites should be between 2 and p+1
    model.addConstrs((u[h] >= 2 for h in DH[1:]), name="lowest_hub_order")
    model.addConstrs((u[h] <= p+1 for h in DH[1:]), name="lowest_hub_order_2")
    # (14) - respecting the tour order
    model.addConstrs(((u[h1] - u[h2] + 1) <= (p)*(1 - y[h1, h2])
                        for h1 in range(2,p+1) for h2 in range(2,p+1) if h1 != h2),
                        name= "respect_order") # DH should be replaced by the sum of the open hubs

    # (16) - if the hub is closed then u should be zero
    model.addConstrs((u[h] <= (p+1)*o[h-1] for h in DH[1:]), name="no_hub_no_u")

    # (17) - adding constraint 17 from the article for testing
    model.addConstr((gp.quicksum(o[h] for h in H) == p), name="constraint_17")
    # we shouldn't need constraint 17 if the variable o is activated by a constraint x which is already written

    model.addConstrs((y[dh2, dh1] + y[dh1, dh2] <= 1
                    for dh1 in DH for dh2 in DH if dh1 != dh2), name="no_subtours_3")
    model.addConstrs((gp.quicksum(y[dh1, dh2] for dh1 in DH if dh1 != dh2)
                                <= 1 for dh2 in DH), name="no_subtours_1")
    model.addConstrs((gp.quicksum(y[dh2, dh1] for dh1 in DH if dh1 != dh2)
                                <= 1 for dh2 in DH), name="no_subtours_2")


    #####################################################
    # 4. Objective function
    #####################################################

    # 1. truck distance cost
    cost_truck_var = gp.quicksum(cost_matrix_1[dh1-1,dh2-1]*c_truck_distance*y[dh1, dh2]
                                 for dh1 in DH[1:] for dh2 in DH[1:] if dh1 != dh2)

    # 2. truck fixed cost
    cost_truck_fixed = c_truck*t # this will change in v2

    model.setObjective(cost_truck_var + cost_truck_fixed)

    #####################################################
    # 5. Saving LP model (remember to change versions)
    #####################################################
    model.write("../output/lp_model/2EVRP_truck_u.lp")

    #### Optimize!
    model.optimize()

    status = model.Status
    if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
        print("Model is either infeasible or unbounded.")
        sys.exit(0)
    elif status != GRB.OPTIMAL:
        print("Optimization terminated with status {}".format(status))
        sys.exit(0)

    #####################################################
    # 6. analyzing solutions
    #####################################################
    print('Optimal solution found with total cost: %g' % model.objVal)

    for v in model.getVars():
        if v.x >= 0.5:
            print('%s %g' % (v.varName, v.x))

    # 1. analyzing hubs
    print("Analyzing hubs")

    # 2. analyzing robots
    print("Analyzing Robots")

    # 3. analyzing tours (echelon 1)
    print("Analyzing tours (depot - hubs)")

    # 4. analyzing tours (echelon 2)
    print("Analyzing tours (hubs - clients)")


    # TODO: move to solution printer
    # solution_printer.print_solution_x(x)
    # solution_printer.print_solution_y(y)
    # solution_printer.print_solution_r(r)

    # model.addConstrs((y[dh2, dh1] + y[dh1, dh2] <= 1
    #                  for dh1 in DH for dh2 in DH if dh1 != dh2), name="no_subtours_3")
    # model.addConstrs((gp.quicksum(y[dh1, dh2] for dh1 in DH if dh1 != dh2)
    #                              <= 1 for dh2 in DH), name="no_subtours_1")
    # model.addConstrs((gp.quicksum(y[dh2, dh1] for dh1 in DH if dh1 != dh2)
    #                       <= 1 for dh2 in DH), name="no_subtours_2")

    # (16 - Extra) - maximum truck capacity - needs correction
    #model.addConstrs(((gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C for r in R for h in H))
    #                 <= V_cap*(gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH for dh2 in DH if dh1 != dh2))),
    #               name="truck_cap")

    #########################################################""

def create_model_5(depot, hubs, clients,
                       cost_matrix_1, cost_matrix_2):
    # completely different
    pass