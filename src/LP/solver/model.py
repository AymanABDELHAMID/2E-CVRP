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

def create_model_1(vans, robots, depot, hubs, clients,
                   cost_matrix_1, cost_matrix_2, cost_matrix_3):
    """
    Model 1 = A simple 2E-CVRP with no time window constraints

    :param vans:
    :param robots: an object with location and number
    :param depot: location of the depot station, name of the depot
    :param hubs: each hub has a location
    :param clients: clients have names,
    :param cost_matrix_1: a dict that uses the keys of locations to give you the distance (ex. cost_matrix_1[h.name,d.name] = distance between hub i and depot j)
    :param cost_matrix_2: a dict that uses the keys of locations to give you the distance
    :param cost_matrix_3: a dict that uses the keys of locations to give you the distance
    :return:  Gurobi model with the related constraints
    """
    ### Creating model:
    model = gp.Model("2E-VRP")

    #####################################################
    # 1. sets & parameters (parameters should be already defined in __init__)
    #####################################################

    #####################################################
    # 2. decision variables
    #####################################################


    #####################################################
    # 3. constraints
    #####################################################

    #### Optimize!

    # solution_printer.print_solution_x(x)
    # solution_printer.print_solution_y(y)
    # solution_printer.print_solution_r(r)


