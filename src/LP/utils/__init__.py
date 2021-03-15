# This file will contain the entry point where you load the data and init the variables

## imports
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import pandas as pd
import csv
import io


# print('⚡️Reading the problem instances... ')

class data(object):
    """
    A class to hold the instance data.
    """
    def __init__(self, type=None):
        self.node_dict = {}
        self.num_customers = 0
        self.stops = {}
        self.satellites = {}
        self.depots = {}
        self.type = type

# TODO: add configurations and params here
