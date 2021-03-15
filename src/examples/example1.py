"""
Groupe 2 - Transport & Logistiques
Modeling the 2 echelon  VRP Gurobi Solver - Mathematical formulation is inspired from :
A two-tier urban delivery network with robot-based deliveries - Bakach et al. 2019

Mars 2021

example 1 : testing reading instances
"""

from ...src.LP.utils import data_reader as dr

def main():
    path = "../../instances/LV-SAV-instances/3-15/Ca1-3,15.txt"
    dr.read_d(path).head()

if __name__ == '__main__':
    main()