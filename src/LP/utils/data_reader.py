"""
Groupe 2 - Transport & Logistiques
Modeling the 2 echelon  VRP Gurobi Solver - Mathematical formulation is inspired from :
A two-tier urban delivery network with robot-based deliveries - Bakach et al. 2019

Mars 2021
"""
import pandas as pd
import io
import csv
import numpy as np
import math

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

class Client():
    def __init__(self, name, loc_x, loc_y, demand):
        self.name = name
        self.loc = [loc_x, loc_y]
        self.demand = demand

    def __str__(self):
        return f"Client: {self.name}.\n  Location: {self.loc}.\n  Demand: {self.demand} units."
# TODO: add configurations and params here


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
    distance = math.sqrt(math.pow((loc1[0]-loc2[0]),2) + math.pow((loc1[1]-loc2[1]),2))
    return distance

def read_clients(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    #clients = df.loc[]
    clients = df.loc[df.demand.notna()]
    return clients.values

def read_clients_loc_only(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    #clients = df.loc[]
    clients_temp = df.loc[df.demand.notna()]
    clients = clients_temp.drop(['demand'], axis=1)
    return clients.values

def create_client_obj(client_list):
    """

    :param clients:
    :return: a list of client objects
    """
    clients = []
    for i, data in enumerate(client_list): # advantage, you can add names to customers
            client_temp = Client(name = str(i), loc_x= data[0], loc_y=data[1], demand=data[2])
            clients.append(client_temp)
    return clients

def read_depot(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    #clients = df.loc[]
    depot = df.loc[df.demand.isna(), ["x_loc", "y_loc"]]
    return depot.values[-1]

def read_hubs(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    #clients = df.loc[]
    hubs = df.loc[df.demand.isna(),["x_loc","y_loc"]]
    return hubs.values[:-1]


def read_dep_hubs(df):
    """
    reads the instance with the data, return the clietns data only
    :param dataframe:
    :return: a list of clients with their locations, demand
    """
    dep_hub = df.loc[df.demand.isna(),["x_loc","y_loc"]]
    return dep_hub.values

def cost_matrix_multiple_depot(depots, hubs):
    """
    Reads location data of depot and hubs.
    creates cost matrix between satellite hubs and customers
    :param depots_hubs: use read_dep_hubs to get the necessary input from the instance dataframe
    :return:
    """
    cost_matrix_1 = {(d, h): 0 for d in range(len(depots)) for h in range(len(hubs))}
    for i, loc1 in enumerate(depots):
        for j, loc2 in enumerate(hubs):
            cost_matrix_1[i,j] = compute_distance(loc1, loc2)
    return cost_matrix_1

def cost_matrix_1_single_depot(depots, hubs):
    """
    This is just the distance from depot to hubs
    Reads location data of depot and hubs.
    creates cost matrix between satellite hubs and customers
    :param depots_hubs: use read_dep_hubs to get the necessary input from the instance dataframe
    :return:
    """
    depots = [depots]
    cost_matrix_1 = {(d, h): 0 for d in range(len(depots)) for h in range(len(hubs))}
    for i, loc1 in enumerate(depots):
        for j, loc2 in enumerate(hubs):
            cost_matrix_1[i,j] = compute_distance(loc1, loc2)
    return cost_matrix_1


def cost_matrix_1(depots_hubs):
    """
    The poblem with the one is that it doesn't follow the same indexing structure.
    But you will have to use this one for now.
    Reads location data of depot and hubs.
    creates cost matrix between satellite hubs and customers
    :param depots_hubs: use read_dep_hubs to get the necessary input from the instance dataframe
    :return:
    """
    cost_matrix_1 = {(d, h): 0 for d in range(len(depots_hubs)) for h in range(len(depots_hubs))}
    for i, loc1 in enumerate(depots_hubs):
        for j, loc2 in enumerate(depots_hubs):
            if i < j:
                cost_matrix_1[i,j] = compute_distance(loc1, loc2)
                cost_matrix_1[j, i] = cost_matrix_1[i,j]
    return cost_matrix_1

def cost_matrix_2(hubs, clients): # not used
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
            #cost_matrix_2[j, i] = cost_matrix_2[i, j]
    return cost_matrix_2

def cost_matrix_3(clients_loc): # not used
    """
    Reads location data of clients and hubs.
    cost matrix between clients (it is symmetrical)

    :param clients:
    :return: a dict that takes number of client 1 and number of client 2 as keys to give you the distance between both,
    """
    cost_matrix_3 = {(c1, c2): 0 for c1 in range(len(clients_loc)) for c2 in range(len(clients_loc))}
    for i, client_loc1 in enumerate(clients_loc):
        for j, client_loc2 in enumerate(clients_loc):
            if i < j:
                cost_matrix_3[i, j] = compute_distance(client_loc1, client_loc2)
                cost_matrix_3[j, i] = cost_matrix_3[i, j]
    return cost_matrix_3

def cost_matrix_4(clients_loc, hubs):
    """
    After the new formulation we don't need cost_matrix_2 or ""_3. We will keep them for now
    Reads location data of clients and hubs.
    cost matrix between clients (it is symmetrical)
    the length
    :param clients:
    :return: a dict that takes number of client 1 and number of client 2 as keys to give you the distance between both,
    """
    clients_hubs = np.concatenate((clients_loc,hubs), axis = 0)
    cost_matrix_4 = {(i, j): 0 for i in range(len(clients_hubs)) for j in range(len(clients_hubs))}
    for i, loc1 in enumerate(clients_hubs):
        for j, loc2 in enumerate(clients_hubs):
            if i < j:
                cost_matrix_4[i, j] = compute_distance(loc1, loc2)
                cost_matrix_4[j, i] = cost_matrix_4[i, j]
    return cost_matrix_4