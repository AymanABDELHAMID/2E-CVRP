"""
Trying a different approach
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

def create_model_FLP(hubs, clients, cost_matrix_2):
    """
    Model to minimize the number of hubs to open following the classical facility location problem.
    Maybe we will not need this part
    :param hubs:
    :param clients:
    :param cost_matrix_2:
    :return: hub that are open.
    """

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
    R_max = 5 #len(clients) #TODO: The number of solutions
    R = list(range(R_max))  # list of robots
    C = [c.name for c in clients]  # list of clients
    D_c = [c.demand for c in clients]  # list of demands
    # time windows and duration:
    st = {c.name: c.st for c in clients}
    t1 = {c.name: c.tw1 for c in clients}
    t2 = {c.name: c.tw2 for c in clients}
    Loc_c = [c.loc for c in clients]  # list of locations
    Loc_c_dict = {c.name: c.loc for c in clients}  # dict of locations - will not be used most probably
    L = list(set([l[0] for l in cost_matrix_2.keys()]))  # L = m+n
    H = list(range(len(hubs)))  # list of hubs (hubs 0,1,2)
    R_cap = 50  # maximum robot capacity
    R_dist = 200
    # fixed costs
    c_hub = 50
    c_robot = 10
    # set upper bound to the last operation last time window
    op_time = max(t2.values()) + 15

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

    # Start time of service
    # TODO: Get the maximum latest time window
    t = model.addVars(L, ub=op_time, name="t") # for instance 1, the maximum time is 263

    # Artificial variables to correct time window upper and lower limits
    xa = model.addVars(C, name="xa")
    xb = model.addVars(C, name="xb")

    #####################################################
    # 3. constraints
    #####################################################

    # A robot must be assigned to a client
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R for h in H) == 1 for c in C), name="client_must_be_served")
    # (4) - if robot serves a client, the corresponding hub must be open
    model.addConstrs((gp.quicksum(x[c, r, h] for r in R) <= o[h] for c in C for h in H),
                     name="no_hub_no_robot")
    model.addConstrs((rob[r, h] <= o[h] for r in R for h in H), name="no_hub_no_robot_2")
    # [2] - At most one robot can be assigned to a job
    model.addConstrs((x.sum(c, '*', h) <= 1 for c in C for h in H), name="one_robot")
    # Robot serves from a single hub:
    # (D) - each client is served by a single robot from a single hub
    model.addConstrs((gp.quicksum(x[c, r, h] for h in H for r in R) == 1 for c in C), name="Just_one_hub")
    # (C) - Robot maximum capacity constraint
    model.addConstrs((gp.quicksum(D_c[int(c)] * x[c, r, h] for c in C for h in H) <= R_cap * rob.sum(r, '*') for r in R),
                     name="robot_cap")
    # (A) - distance covered by every robot should be less than maximum allowed distance
    model.addConstrs((gp.quicksum(cost_matrix_2[i, j] * y[i, j, r] for i in L for j in L if i != j)
                      <= R_dist for r in R), name="robot_dist")
    # Robot tour constraints
    model.addConstrs((gp.quicksum(y[int(j), int(i), r] for j in L if int(j) != int(i))
                      == gp.quicksum(x[i, r, h] for h in H) for r in R for i in C),
                        name="Tour1")
    model.addConstrs((gp.quicksum(y[int(i), int(j), r] for j in L if int(j) != int(i)) == gp.quicksum(x[i, r, h] for h in H) for r in R for i in C),
                 name="Tour2")
    # Additional Tour Constraints:
    model.addConstrs((y[int(j), int(i), r] + y[int(i), int(j), r] <= 1 for r in R
                      for i in L for j in L if int(j) != int(i)), name="Tour3")

    # Ensuring Tours in "y" are respecting the same hub the robot is serving from
    model.addConstrs((gp.quicksum(y[int(c), n_c + h, r] for c in C) == rob[r, h] for r in R for h in H),
                 name="sameHub1")
    model.addConstrs((gp.quicksum(y[n_c + h, int(c), r] for c in C) == rob[r, h] for r in R for h in H),
     name="sameHub2")

    # Temporal constraints for client locations (here we assume distance and time have 2:1 ratio)
    #M = {(i, j): op_time + st[i] + (cost_matrix_2[int(i), int(j)]/2) for i in C for j in C}
    #model.addConstrs((t[int(j)] >= t[int(i)] + st[i] + (cost_matrix_2[int(i), int(j)]/2)
    #              - M[i, j] * (1 - gp.quicksum(y[int(i), int(j), r] for r in R))
    #              for i in C for j in C), name="tempoClients")

    # Temporal constraints for hub locations
    #M = {(i, j): op_time + (cost_matrix_2[int(i), int(j)]/2) for i in H for j in C}
    #model.addConstrs((t[int(j)] >= t[int(i)] + (cost_matrix_2[int(i), int(j)]/2)
    #              - M[i, j] * (1 - gp.quicksum(y[int(i), int(j), r] for r in R)) for i in H for j in C),
    #             name="tempoHub")
    # Time window constraints
    #model.addConstrs((t[int(c)] + xa[c] >= t1[c] for c in C), name="timeWin1")
    #model.addConstrs((t[int(c)] - xb[c] <= t2[c] for c in C), name="timeWin2")

    # Robot serves from one hub only
    model.addConstrs((gp.quicksum(rob[r, h] for h in H) <= 1 for r in R), name="one_hub_per_robot")

    #########################################################
    # 2nd part - Truck assignment
    #########################################################
    # defining the number of trucks by calculating the total demand
    V_cap = 300  # Truck Max Capacity

    total_demand = math.fsum(c.demand for c in clients)
    t = math.ceil((total_demand / V_cap))
    V = list(range(t))  # list of trucks

    c_truck = 50 # Truck Fixed Cost
    c_truck_distance = 5 # Truck Variable Cost

    DH = list(range(len(hubs) + 1)) # List of Depot and Hubs
    z = model.addVars(V, DH, DH, vtype=GRB.BINARY, name="z")

    # (8) - all the trucks must return to the depot station
    # model.addConstr((gp.quicksum(y[h, 0] for h in DH) == 1), name="trucks1")
    # (9) - all trucks must depart from the depart station
    # model.addConstr((gp.quicksum(y[0, h] for h in DH) == 1), name="trucks2")
    # (10) - mirroring Constraint 11
    model.addConstrs((gp.quicksum(z[v, h1, h2] for v in V for h1 in DH if h1 != h2)
                      == gp.quicksum(z[v, h2, h1] for v in V for h1 in DH if h1 != h2)
                      for h2 in DH), name="trucks3")
    # (11) - sum of all trucks going to the same hub is 1 if hub is open
    model.addConstrs((gp.quicksum(z[v, h1, h2] for v in V for h1 in DH if h1 != h2)
                      == o[h2] for h2 in DH[:-1]), name="if_truck_then_hub")
    # Eliminating Subtours

    model.addConstrs((gp.quicksum(z[v, dh1, dh2] for v in V for dh1 in DH if dh1 != dh2)
                      <= 1 for dh2 in DH), name="no_subtours_1")
    model.addConstrs((gp.quicksum(z[v, dh2, dh1] for v in V for dh1 in DH if dh1 != dh2)
                      <= 1 for dh2 in DH), name="no_subtours_2")
    model.addConstrs((z[v, dh2, dh1] + z[v, dh1, dh2] <= 1
                      for v in V for dh1 in DH for dh2 in DH if dh1 != dh2), name="no_subtours_3")

    # Demand per Hub
    D_h = model.addVars(H, name="D_h")
    model.addConstrs((D_h[h] == gp.quicksum(D_c[int(c)]*x[c, r, h] for c in C for r in R) for h in H), name="total_demand_per_hub")
    # Maximum Truck Capacity
    model.addConstrs((gp.quicksum(D_h[h]*z[v, i, h] for i in DH) <= V_cap for h in H for v in V),
                        name="truck_capacity")

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
    cost_truck_fixed = c_truck * t  # this will change in v2



    model.setObjective(cost_robot_var + cost_robot_fixed
                       + cost_hub_fixed + cost_truck_fixed
                       + cost_truck_var, GRB.MINIMIZE)

    #####################################################
    # 5. Saving LP model (remember to change versions)
    #####################################################
    model.write("../output/lp_model/RAP_TRP_v1.lp")

    #### Optimize!
    model.optimize()

    print('Optimal solution found with total cost: %g' % model.objVal)

    for v in model.getVars():
        if v.x >= 0.5:
            print('%s %g' % (v.varName, v.x))

    ### Printing execution Time:
    print("Executed in %s Minutes" % ((time.time() - start_time)/60))

def create_model_TRP(depot, hubs, cost_matrix_1):
    """
    Model tp optimize truck routing from central depot to hubs rspecting:
        1. Hubs demands
        2. Truck maximum capacity
    :param depot:
    :param hubs:
    :param cost_matrix_1:
    :return:
    """