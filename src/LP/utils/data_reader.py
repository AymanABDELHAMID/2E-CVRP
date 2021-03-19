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

def cost_matrix_1(depot, hubs):
    """
    Reads location data of depot and hubs.
    creates cost matrix between satellite hubs and customers
    :param depot:
    :param hubs:
    :return:
    """
    cost_matrix_1 = {(d, h): 0 for d in range(len(depot)) for h in range(len(hubs))}
    for i, depot_loc in enumerate(depot):
        for j, hub_loc in enumerate(hubs):
            cost_matrix_1[i,j] = compute_distance(depot_loc, hub_loc)
            cost_matrix_1[j, i] = cost_matrix_1[i,j]
    return cost_matrix_1

def cost_matrix_2(hubs, clients):
    """
    Reads location data of clients and hubs.
    cost matrix between satellite hubs and clients

    :param hubs:
    :param clients:
    :return:
    """
    cost_matrix_2 = {(d, h): 0 for d in range(len(clients)) for h in range(len(hubs))}
    for i, client_loc in enumerate(clients):
        for j, hub_loc in enumerate(hubs):
            cost_matrix_2[i, j] = compute_distance(client_loc, hub_loc)
            cost_matrix_2[j, i] = cost_matrix_1[i, j]
    return cost_matrix_2

def cost_matrix_3(clients):
    """
    Reads location data of clients and hubs.
    cost matrix between clients (it is symmetrical)

    :param clients:
    :return:
    """
    cost_matrix_3 = {(d, h): 0 for d in range(len(clients)) for h in range(len(clients))}
    for i, client_loc1 in enumerate(clients):
        for j, client_loc2 in enumerate(clients):
            if i < j:
                cost_matrix_3[i, j] = compute_distance(client_loc1, client_loc2)
                cost_matrix_3[j, i] = cost_matrix_3[i, j]
    return cost_matrix_3