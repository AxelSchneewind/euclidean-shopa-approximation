#!python

# needs to be in PYTHONPATH
import model.model as bench

import pandas as pd
from matplotlib import pyplot as plt

import argparse
import sys



def filter(data):
    # check that cost value is finite
    invalid_cost = data.loc[(data['cost'] == math.inf) | (data['cost'] == math.nan)]
    if len(invalid_cost) != 0:
        print('ignored by cost value: ', file=sys.stderr)
        print(invalid_cost, file=sys.stderr)
    data = data.loc[(data['cost'] != math.inf) & (data['cost'] != math.nan)]

    # check that exact value exists
    has_reference = np.array([reference(data, row).shape[0] != 0 for i, row in data.iterrows()], dtype='bool')
    if len(has_reference[(has_reference == False)]) != 0:
        print('exact value missing for some queries', file=sys.stderr)
        print(data.loc[has_reference == False], file=sys.stderr)
    data = data.loc[has_reference]

    # check that coordinates of source and target coordinates match
    coords_match = np.array([
        (reference(data, row)['source latitude'] == row['source latitude'])
        & (reference(data, row)['source longitude'] == row['source longitude'])
        & (reference(data, row)['target latitude'] == row['target latitude'])
        & (reference(data, row)['target longitude'] == row['target longitude']).all() for i,row in data.iterrows() ], dtype='bool')
    if len(coords_match[(coords_match == False)]) != 0:
        print('mismatch in source/target coordinates', file=sys.stderr)
        print(data.loc[coords_match == False], file=sys.stderr)
    data = data.loc[coords_match]

    return data




def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True, action='append', help='path to the input file')
    parser.add_argument('--column', '-c', default='time', help='the column to show box plots of')
    args = parser.parse_args()

    # load data and filter out unusable results
    data = bench.load(args.file)
    data = filter(data)

    # by epsilon
    columns_by_epsilon = [ data[data['epsilon'] == eps][args.column] for eps in data['epsilon'].unique() ]

    # generate box plots
    fig, ax = plt.subplots()
    ax.boxplot(columns_by_epsilon, labels=data['epsilon'].unique())
    ax.set_xlabel('$\\varepsilon$')
    ax.set_ylabel(args.column)

    plt.show()

 
if __name__ == "__main__":
    main()
