#!python

# needs to be in PYTHONPATH
import model as bench

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import math
import argparse
import sys



def filter(data):
    # check that exact value exists
    has_reference = np.array([bench.reference(data, row).shape[0] != 0 for i, row in data.iterrows()], dtype='bool')
    if len(has_reference[(has_reference == False)]) != 0:
        print('exact value missing for some queries', file=sys.stderr)
        print(data.loc[has_reference == False], file=sys.stderr)
    data = data.loc[has_reference]

    # check that coordinates of source and target coordinates match
    coords_match = np.array([
        ((bench.reference(data, row)['source latitude'] == row['source latitude'])
        & (bench.reference(data, row)['source longitude'] == row['source longitude'])).all() for i,row in data.iterrows() ], dtype='bool')
    if len(coords_match[(coords_match == False)]) != 0:
        print('mismatch in source/target coordinates', file=sys.stderr)
        print(data.loc[coords_match == False], file=sys.stderr)
    data = data.loc[coords_match]

    print(len(data), 'usable rows')
    return data




def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True, action='append', help='path to the input file')
    parser.add_argument('--column', '-c', default='time', help='the column to show box plots of')
    parser.add_argument('--fliers', action='store_true', help='whether to show fliers on boxplots')
    parser.add_argument('--means', action='store_true', help='whether to show means on boxplots')
    parser.add_argument('--graph-type', '-t', default='triangle')
    parser.add_argument('--output-file', '-o', default='out.png')
    args = parser.parse_args()

    column_unit = bench.column_units[args.column]

    # load data and filter out unusable results
    data = bench.load(args.file)

    # compare given graph type to exact values
    graph_type = '-' + args.graph_type
    data = data.loc[(data['benchmark'].str.contains(graph_type)) | (data['benchmark'].str.contains('-exact'))]
    print('using benchmarks: ', data['benchmark'].unique())

    data = filter(data)

    data.sort_values(by='epsilon', inplace=True)
    data.sort_index(inplace=True, ignore_index=True)
    data.sort_values(by='epsilon', inplace=True)

    print('usable data contains ', len(np.unique(data[['source','target']], axis=0)), ' entries')

    # by epsilon
    column_by_epsilon = [ data.loc[data['epsilon'] == eps, args.column].to_numpy(dtype='float') for eps in data['epsilon'].unique() ]

    # generate box plots
    fig, ax = plt.subplots()
    ax.boxplot(column_by_epsilon, labels=data['epsilon'].unique(), showfliers=args.fliers, showmeans=args.means)
    ax.set_xlabel('$\\varepsilon$')
    ax.set_ylabel(args.column + ' [' + column_unit + ']')

    plt.savefig(args.output_file)
    # plt.show()

 
if __name__ == "__main__":
    main()
