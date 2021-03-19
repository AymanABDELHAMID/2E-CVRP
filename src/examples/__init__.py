from ...src.LP.utils import data_reader as dr

def main():
    path = "../../instances/LV-SAV-instances/3-15/Ca1-3,15.txt"  # remember the path should be relative to the data reader
    # d = dr.read_d(path)
    d = dr.read_d_pandas_2(path)
    print(d)
