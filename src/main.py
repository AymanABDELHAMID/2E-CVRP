from src.LP.utils import data_reader as dr


path = "../../instances/LV-SAV-instances/3-15/Ca1-3,15.txt" #remember the path should be relative to the data reader
d = dr.read_d(path)
print(d)