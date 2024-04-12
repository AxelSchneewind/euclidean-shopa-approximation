#!python

# needs to be in PYTHONPATH
import model as bench

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import math
import argparse
import sys



def filter_data(data):
    # check that coordinates of source and target coordinates match
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True, action='append', help='path to the input file')
    parser.add_argument('--column', '-c', default='time', help='the column to show box plots of')
    parser.add_argument('--fliers', action='store_true', help='whether to show fliers on boxplots')
    parser.add_argument('--means', action='store_true', help='whether to show means on boxplots')
    args = parser.parse_args()

    # load data and filter out unusable results
    data = bench.load(args.file)

    # 
    data = data.loc[(data['benchmark'].str.contains('-unref') == False)]
    print(data['benchmark'].unique())

    data = filter_data(data)

    data.sort_values(by=['benchmark', 'epsilon'], inplace=True)
    data.sort_index(inplace=True, ignore_index=True)
    data.sort_values(by=['benchmark', 'epsilon'], inplace=True)

    print('usable data contains ', len(np.unique(data[['source','target']], axis=0)), ' entries')
    print(data)


    # by benchmark
    plots = [(b,eps) for eps in data['epsilon'].unique() for b in data['benchmark'].unique() ]
    plots = [p for p in filter(lambda x: len(data.loc[(data['benchmark'] == x[0]) & (data['epsilon'] == x[1])]) > 0, plots)]

    column_by_epsilon = [ data.loc[(data['benchmark'] == b) & (data['epsilon'] == eps), args.column] for (b,eps) in plots ]
    print(len(column_by_epsilon), len(plots))
    # generate box plots
    fig, ax = plt.subplots()
    ax.boxplot(column_by_epsilon, labels=[ str(p[0]) + ', $\\varepsilon = ' + str(p[1]) + '$' for p in plots ], showfliers=args.fliers, showmeans=args.means)
    ax.set_xlabel('benchmark')
    if args.column in bench.column_units and bench.column_units[args.column] != "":
        ax.set_ylabel(args.column + '[' + bench.column_units[args.column] + ']')
    else:
        ax.set_ylabel(args.column)

    plt.show()

 
if __name__ == "__main__":
    main()
