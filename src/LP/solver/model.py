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

def create_model_1(vans, depot, hubs, clients,
                   cost_matrix_1, cost_matrix_2):
    """
    Model 1 = A simple 2E-CVRP with no time window constraints

    :param vans:
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
    R_max = len(clients)
    R = list(range(R_max)) # list of robots
    C = [c.name for c in clients] # list of clients
    D_c = [c.demand for c in clients] # list of demands
    L = list(set([l[0] for l in cost_matrix_2.keys()])) # L = m+n
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
    n_c = len(clients) # number of customers

    #####################################################
    # 2. decision variables
    #####################################################
    # hub open/not-open
    o = model.addVars(H, vtype=GRB.BINARY, name="o")
    # robot-client-hub assignment
    x = model.addVars(C, R, H, vtype=GRB.BINARY, name="x")
    # robot assignment
    r = model.addVars(R, vtype=GRB.BINARY, name="r")
    # van-hub assignment
    # TODO: compare with the TSP formulation
    # create a list of Depots + Hubs
    # DH = [0]+H # in our case it is only one depot, the model is extendable to include multiple (if we..)
    DH = list(range(len(hubs)+1))
    # TODO: remember in the distance matrix hubs are 1,2,3
    # y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y")
    y = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="y")
    # edges assignment to robots
    z = model.addVars(L, L, R, vtype=GRB.BINARY, name="z")

    #####################################################
    # 3. constraints
    #####################################################
    # (2) - all client nodes are assigned to hubs
    # implemented in constraint (D)
    # (4) - if robot serves a client, the corresponding hub must be open
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R) <= o[h] for c in C for h in H),
                     name="no_hub_no_robot")
    # (8) - all the trucks must return to the depot station
    model.addConstrs((gp.quicksum(y[v, dh, 0] for dh in DH[1:]) == t for v in V), name="trucks1")
    # (9) - all trucks must depart from the depart station
    model.addConstrs((gp.quicksum(y[v, 0, dh] for dh in DH[1:]) == t for v in V), name="trucks2")
    # (10) - number of trucks leaving = number of trucks returning
    # TODO: review indexing
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH[1:] if dh1 != dh2)
                      == gp.quicksum(y[v, dh2, dh1] for v in V for dh1 in DH[1:] if dh1 != dh2) for dh2 in DH[1:]),
                        name="trucks3")
    # (11) - sum of all trucks going to the same hub is 1 if hub is open
    model.addConstrs((gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH if dh1 != dh2)
                      == o[dh2-1] for dh2 in DH), name="if_truck_then_hub")  #test with putting cstr 11 before 4
    # (16) - maximum truck capacity - needs correction
    #model.addConstrs(((gp.quicksum(D_c[c]*x[c, r, h] for c in C for r in R for h in H))
    #                  <= V_cap*(gp.quicksum(y[v, dh1, dh2] for v in V for dh1 in DH for dh2 in DH))),
    #                 name="truck_cap")

    # (A) - distance covered by every robot should be less than maximum allowed distance
    model.addConstrs((gp.quicksum(cost_matrix_2[c1, c2] * z[c1, c2, r] for c1 in L for c2 in L if c1 != c2)
                      <= R_dist for r in R),
                     name="robot_dist")
    # forming a tour :
        # (B1) - ensure that the robot assigned to a client travels to another location after every client
    model.addConstrs((z.sum('*', c, r) == x[c, r, h] for r in R for c in C for h in H),
                 name="tour1")
    # (B2) - same but from j --> i
    model.addConstrs((z.sum(c, '*', r) == x[c, r, h] for r in R for c in C for h in H),
                 name="tour2")
    # maybe will need to add constraint to the depot.
    # (B3) - ensure that each robot forms a tour from and to the same hub
    # Same hub constraints:
    # TODO: compare between running with B3 and B4 and without
    model.addConstrs((z[c, n_c + h, r] == x[c, r, h] for r in R for c in C for h in H),
                 name="sameHub1")
    # (B4) - same as (B3) but from j --> i
    model.addConstrs((z[n_c + h, c, r] == x[c, r, h] for r in R for c in C for h in H),
                 name="sameHub2")

    # (C) - respect maximum robot capacity
    model.addConstrs((gp.quicksum(D_c[c]*x[c, r, h] for c in C for h in H) <= R_cap for r in R),
                     name="robot_cap")
    # (D) - each client is served by a single robot from a single hub
    model.addConstrs((gp.quicksum(x[c, r, h] for h in H for r in R) == 1 for c in C), name="Just_one_hub")
    # (E) - same as (D) but summing over all clients and all hubs
    model.addConstrs((x.sum(c, '*', h) <= 1 for c in C for h in H), name="Just_one_robot") # test with ==

    # (R) - decision variable r constraints - needs revision
    model.addConstrs(r[r] == gp.quicksum(z[c1, c2, r] for c1 in C for c2 in C if c1 != c2) for r in R)

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
    cost_robot_fixed = gp.quicksum(c_robot*r[r] for r in R)

    # 5. hub fixed cost
    cost_hub_fixed = gp.quicksum(c_hub*o[h] for h in H)

    model.setObjective(cost_truck_var + cost_truck_fixed +
                       cost_robot_var + cost_robot_fixed
                       + cost_hub_fixed, GRB.MINIMIZE)

    #####################################################
    # 5. Saving LP model (remember to change versions)
    #####################################################
    model.write("../../output/lp_model/2EVRP_v1.lp")

    #### Optimize!
    model.optimize()

    status = model.Status
    if status in [GRB.INF_OR_UNBD, GRB.INFEASIBLE, GRB.UNBOUNDED]:
        print("Model is either infeasible or unbounded.")
        sys.exit(0)
    elif status != GRB.OPTIMAL:
        print("Optimization terminated with status {}".format(status))
        sys.exit(0)

    # solution_printer.print_solution_x(x)
    # solution_printer.print_solution_y(y)
    # solution_printer.print_solution_r(r)


