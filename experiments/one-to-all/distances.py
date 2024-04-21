#!python3

import sys
import numpy as np
import pandas as pd


def read_distances(file):
    data = pd.read_csv(file, dtype={ 'distance': np.float64, 'node':np.int64 })
    data.sort_values(by='distance', ascending=True, inplace=True)
    data.drop_duplicates(subset=['node'], keep='first', inplace=True, ignore_index=True)
    return data

def add_ratio(data, reference):
    distance_column = reference.columns[1]
    data['ratio'] = data[distance_column] / reference[distance_column]
    return data

def describe(file, reference_file, output):
    data = read_distances(file)
    data_ref = read_distances(reference_file)

    add_ratio(data, data_ref)

    description = data[['ratio']].describe().T
    description['file'] = file
    description['reference'] = reference_file
    print(description)
    description.to_csv(output)


if __name__ == '__main__':
    if len(sys.argv) > 3:
        describe(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print('usage: ', sys.argv[0], 'distances.csv reference_distances.csv quality.csv')
