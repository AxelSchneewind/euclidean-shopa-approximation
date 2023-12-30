#!python3


from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

import sys



def read(file):
    return pd.read_csv(file)


def plot(data, x_column, y_column):
    data.sort_values(x_column, inplace=True)
    #data.sort_values(y_column, inplace=True)

    x_values = data[x_column]
    y_values = data[y_column]

    plt.scatter(x_values, y_values)
    #plt.scatter(range(len(x_values)), y_values)
    plt.xlabel(x_column)
    plt.ylabel(y_column)
    plt.show()

def plot_sorted(data, column):
    data.sort_values(column, inplace=True)

    y_values = data[column]
    x_values = range(len(y_values))

    plt.scatter(x_values, y_values)
    plt.ylabel(column)
    plt.show()



def plot_cumulative(data, x_column, y_column):
    data.sort_values(x_column, inplace=True)

    x_values = data[x_column]
    y_values = data[y_column]

    for i in range(1, len(y_values)):
        y_values[i] += y_values[i-1]

    plt.scatter(x_values, y_values)
    plt.xlabel(x_column)
    plt.ylabel(y_column + ' cumulative')
    plt.show()





if __name__ == "__main__":
    if len(sys.argv) < 4:
        exit(-1);

    file = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]

    data = read(file)
    plot(data, x, y)
    # plot_sorted(data, y)

