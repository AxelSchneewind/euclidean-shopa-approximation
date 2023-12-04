#!/bin/python


import pandas as pd
import sys



# reads a csv file with columns: node,distance and outputs a text file where line i contains the distance of node i
if __name__ == "__main__":
    if len(sys.argv) < 2:
        exit();

    data = pd.read_csv(sys.argv[1])
    data.drop_duplicates(subset='node', inplace=True)
    data.sort_values('node', inplace=True)

    data.to_csv(sys.argv[1], columns=['distance'], index=False, header=False)
    
