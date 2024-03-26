#!python

# module needs to be in PYTHONPATH
import model.model as bench

import argparse
import sys
import matplotlib
from matplotlib import pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True, action='append', help='path to the input file')
    parser.add_argument('--column', '-c', default='time', help='the column to show box plots of')
    args = parser.parse_args()

    # load
    data = bench.load(args.file)

    # group by benchmark
    columns_by_algorithm = [ data[data['benchmark'] == b][args.column] for b in data['benchmark'].unique() ]

    # generate box plots
    fig, ax = plt.subplots()
    ax.boxplot(columns_by_algorithm, labels=data['benchmark'].unique())
    ax.set_xlabel('algorithm')
    ax.set_ylabel(args.column)

    plt.show()

 
if __name__ == "__main__":
    main()
   