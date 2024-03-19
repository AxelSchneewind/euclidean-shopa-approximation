#!python


import summarize as s
import pandas as pd
from matplotlib import pyplot as plt

import argparse
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True, action='append', help='path to the input file')
    parser.add_argument('--column', '-c', default='time', help='the column to show box plots of')
    args = parser.parse_args()


    # load data and filter out unusable results
    data = s.load(args.file)
    data = s.filter(data)

    # derive new columns from existing ones
    data = s.add_optimal_cost(data)
    data = s.add_ratios(data)

    # by epsilon
    columns_by_epsilon = [ data[data['epsilon'] == eps][args.column] for eps in data['epsilon'].unique() ]

    # generate box plots
    fig, ax = plt.subplots()
    ax.boxplot(columns_by_epsilon, labels=[eps for eps in data['epsilon'].unique()])
    ax.set_xlabel('$\\varepsilon$')
    ax.set_ylabel(args.column)

    plt.show()

 
if __name__ == "__main__":
    main()
