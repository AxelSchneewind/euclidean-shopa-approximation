#!python


from matplotlib import pyplot as plt
import pandas as pd
import sys
import argparse
import numpy as np


def read(filename):
    return pd.read_csv(filename)

def plot_statistic(ax, data, x_column, y_column, functions=['mean']):
    groups = data.groupby(by=x_column, group_keys=True)[[y_column]]
    print(groups.describe())
    label = y_column

    x_values = data[x_column].unique()
    for m in functions:
        y_values = groups.mean()
        print(y_values)
        ax.plot(x_values, y_values, label=label)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='plotter', description='groups data by one column and plots some measures over these groups')
    parser.add_argument('--filename', '-f')
    parser.add_argument('--x-column', '-x')
    parser.add_argument('--y-column', '-y')
    args = parser.parse_args()

    data = read(args.filename)
    fig, ax = plt.subplots(1, 1)
    plot_statistic(ax, data, args.x_column, args.y_column)

    plt.show()
