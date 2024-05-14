#!python3

import sys
import numpy as np
import pandas as pd


def read_distances(file):
    data = pd.read_csv(file, dtype={ 'distance': np.float64, 'node': np.int64 })
    data.sort_values(by='distance', ascending=True, inplace=True)
    data.drop_duplicates(subset='node', keep='first', inplace=True, ignore_index=True)
    data = data.loc[data['distance'] != 0.0]
    data = data.set_index('node')
    return data

def add_ratio(data, reference):
    distance_column = 'distance'
    data = data.join(reference, how='inner', rsuffix='_ref', validate='1:1')
    data['ratio'] = (data['distance'] / data['distance_ref'])
    return data

def describe(file, reference_file, output):
    data = read_distances(file)
    data_ref = read_distances(reference_file)

    # filter by nodes that are contained in both lists
    # keep = [ n in data['node'] for n in data_ref['node'] ]
    # data_ref = data_ref.loc[keep]
    # keep = [ n in data_ref['node'] for n in data['node'] ]
    # data = data.loc[keep]

    # data.reset_index(drop=True)
    # data_ref.reset_index(drop=True)

    data = add_ratio(data, data_ref)
    data.sort_values(by='distance')

    description = data[['ratio']].describe().T
    description['file'] = file
    description['reference'] = reference_file
    description.to_csv(output)


if __name__ == '__main__':
    if len(sys.argv) > 3:
        describe(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print('usage: ', sys.argv[0], 'distances.csv reference_distances.csv quality.csv')
