#!python3

import sys
import numpy as np
import pandas as pd


def read_distances(file):
    data = pd.read_csv(file, dtypes={ 'distance': np.float64, 'node':np.int64 })
    data.sort_values(by='node', inplace=True)
    return data

def add_ratio(data, reference):
    distance_column = reference.columns[1]
    data['ratio'] = data[distance_column] / reference[distance_column]
    return data

def describe(file, reference_file):
    data = read_distances(file)
    data_ref = read_distances(reference_file)

    add_ratio(data, data_ref)

    print(data['ratio'].describe())


if __name__ == '__main__':
    if len(sys.argv) > 2:
        describe(sys.argv[0], sys.argv[1])