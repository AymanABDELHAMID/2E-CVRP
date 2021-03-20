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
import io
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
    R_cap = 50 # maximum robot capacity
    R_dist = 200
    V_cap = 300
    # defining the number of trucks by calculating the total demand
    total_demand = math.fsum(c.demand for c in clients)
    t = math.ceil((total_demand/V_cap))
    V = list(range(t))
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

    # (4) - if robot serves a client, the corresponding hub must be open

    # (8) - all the trucks must return to the depot station

    # (9) - all trucks must depart from the depart station

    # (10) - number of trucks leaving = number of trucks returning

    # (11) - sum of all trucks going to the same hub is 1 if hub is open

    # (A) - distance covered by every robot should be less than maximum allowed distance

    # forming a tour :
        # (B1) - ensure that the robot assigned to a client travels to another location after every client

        # (B2) - same but from j --> i

    # (C) - respect maximum robot capacity

    # (D) - each client is served by a single robot from a single hub

    # (E) - same as (D) but summing over all clients and all hubs

    
    #### Optimize!

    # solution_printer.print_solution_x(x)
    # solution_printer.print_solution_y(y)
    # solution_printer.print_solution_r(r)


