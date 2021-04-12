"""
Ayman Mahmoud - Mars 2021
Group 2 - Hub based model
"""
#from src.LP.solver import model as md
#from src.LP.solver import model_2 as md
#from src.LP.solver import model_3 as md
#from src.LP.solver import model_4 as md
#from src.LP.solver import model_5 as md
from src.LP.solver import model_6 as md
from src.LP.utils import data_reader as dt

# remember the path should be relative to the data reader
#path = "../../instances/LV-SAV-instances/3-15/Ca1-3,15.txt"
path = "../../instances/LV-SAV-instances/3-15/Ca3-3,15.txt"
#path = "../../instances/LV-SAV-instances/3-30/Ca1-3,30.txt"
# 1. convert instance into dataframe
data = dt.read_d_pandas(path)
# this is the instance in a dataframe format - with the data we need: location, demand

# 2. create client data
    # 2.1 - client list
client_list = dt.read_clients(data)
    # 2.2 - client location
client_loc = dt.read_clients_loc_only(data)
    # 2.3 - client object
client_obj = dt.create_client_obj(client_list)

# 3. create hub and depot data
    # 3.1 - hubs
hub_list = dt.read_hubs(data)
    # 3.2 - depot
depot_list = dt.read_depot(data)
    # 3.3 - hub and depot list
hub_depot_list = dt.read_dep_hubs(data)


# 4. create cost matrices
    # 4.1 - cost_matrix_1 (hubs and depots) (n_depots + n_hubs)
c_matrix_1 = dt.cost_matrix_1(hub_depot_list)
    # 4.2 - cost_matrix_2 (hubs and clients) (n_clients + n_hubs)
c_matrix_2 = dt.cost_matrix_4(client_loc, hub_list)


# 6. run the model
#md.create_model_1(depot = depot_list,
#                        hubs = hub_list,
#                        clients = client_obj,
#                        cost_matrix_1= c_matrix_1,
#                        cost_matrix_2= c_matrix_2)

#md.create_model_4(depot = depot_list,
#                        hubs = hub_list,
#                        clients = client_obj,
#                        cost_matrix_1= c_matrix_1,
#                        cost_matrix_2= c_matrix_2)

#md.create_model_RAP(hubs=hub_list,
#                    clients=client_obj,
#                    cost_matrix_2=c_matrix_2)

md.create_model(hubs=hub_list,
                    clients=client_obj,
                    cost_matrix_1=c_matrix_1,
                    cost_matrix_2=c_matrix_2)

# 7. analyze results