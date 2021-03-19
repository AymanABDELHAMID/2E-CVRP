import numpy as np
import gurobipy as gp
from gurobipy import GRB
import pandas as pd
import csv
import io

#####################################################
# 1. parameters
#####################################################


####################################################
# 2. Helper classes
####################################################
class Client():
    def __init__(self, name, loc_x, loc_y, demand):
        self.name = name
        self.loc = [loc_x, loc_y]
        self.demand = demand

    def __str__(self):
        return f"Client: {self.name}.\n  Location: {self.loc}.\n  Demand: {self.demand} units."