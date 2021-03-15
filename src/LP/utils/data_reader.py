"""
Groupe 2 - Transport & Logistiques
Modeling the 2 echelon  VRP Gurobi Solver - Mathematical formulation is inspired from :
A two-tier urban delivery network with robot-based deliveries - Bakach et al. 2019

Mars 2021
"""
import pandas as pd
import io

def read_d(path):
    """
    params: path of the instance of interest

    output: a dataframe with the location and the demand
    """
    #data = open(path, "r")
    data = pd.read_csv(path,
                     delimiter = '\t',#or sep='\t',#, #delim_whitespace=True,  # or sep="\s+" #separator is whitespace
                     header=None) #,  # no header
                     #lineterminator='\r\n') #,
                     #usecols=[1,2,5])
                     #names=['x_loc', 'y_loc', 'demand'])#, engine='python')
    return data

def cost_matrix_1(data):
    """
    cost matrix between satellite hubs and customers
    params: path of the instance of interest

    output:
    """
    pass


def cost_matrix_2(data):
    """
    cost matrix between satellite hubs and depot station
    params: path of the instance of interest

    output:
    """
    pass

def compute_distance(loc1, loc2):
    """
    params:
        - loc1 (location of start vertex)
        - loc2 (location of destination vertex)

    output:
        euclidean distance between the two vertices
    """