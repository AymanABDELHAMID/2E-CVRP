"""
Groupe 2 - Transport & Logistiques
Modeling the 2 echelon  VRP Gurobi Solver - Mathematical formulation is inspired from :
A two-tier urban delivery network with robot-based deliveries - Bakach et al. 2019

Mars 2021
"""
import pandas as pd
import io
import csv

def read_d_pandas(path):
    """
    params: path of the instance of interest

    output: a dataframe with the location and the demand
    """
    #data = open(path, "r")
    data = pd.read_csv(path,
                     sep='\s{2,}', #delimiter = '\t',#or sep='\t',#, #delim_whitespace=True,  # or sep="\s+" #separator is whitespace
                     header=None, #,  # no header
                     lineterminator='\r\n',
                     usecols=[0,1,4],
                     names=['x_loc', 'y_loc', 'demand'],
                     engine='python',
                     thousands=',')
    return data

def read_d_pandas_2(path):
    """

    :param path:
    :return:
    """
    data = pd.read_csv(path, sep='\s{2,}', header=None, engine='python', thousands=',')
    return data

def read_d(path):
    """

    :param path:
    :return:
    """
    x_loc = []
    y_loc = []
    Demand = []

    #with open(path) as data:
    #data.read()
    data = open(path, "r")
    for line in data:
        x, y, t_1, t_2, d, s_t = line.split("\t")
        x_loc.append(x)
        y_loc.append(y)
        Demand.append(d)
    return Demand

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

def read_clients(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    #clients = df.loc[]
    clients = df.loc[df.demand.notna()]
    return clients

def read_depot(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    #clients = df.loc[]
    depot = df.loc[df.demand.isna(), ["x_loc", "y_loc"]]
    return depot.values
    #TODO: only returns the last element

def read_hubs(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    #clients = df.loc[]
    hubs = df.loc[df.demand.isna(),["x_loc","y_loc"]]
    return hubs.values
    #TODO: return all elements except for last element