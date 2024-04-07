#!python3


from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

import sys
import math

plt.rcParams['text.usetex'] = True

def read(file):
    return pd.read_csv(file)


def plot(data, x_column, y_column):
    label = '$|V_{{\\varepsilon={}}}|$'.format(data['epsilon'].iloc[0])

    x_values = data[x_column]
    y_values = data[y_column]
    plt.scatter(x_values, y_values, marker='x', label=label)

# make theoretical upper bound
def plot_theory(data, x_column, epsilon):
    label = '$ D(e), \\varepsilon = {}$ '.format(epsilon)
    #(r'$\frac{1}{\sin(\alpha_{min})}'
    #r'\log\left(\frac{25}{\sin(\alpha_{min})}\right)'
    #r'\cdot\frac{1}{1}\log(\frac 2 1)$')

    x_values = data[x_column].drop_duplicates().sort_values()
    r = [2/s * math.log(25/s) * 1/eps * math.log2(2/eps) for s in x_values]

    plt.plot(x_values, r, label=label)


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print('expected arguments: path/to/data.csv x_column y_column')
        exit(-1)

    file = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]

    data = read(file)

    for eps in data['epsilon'].drop_duplicates():
        d = data[data['epsilon'] == eps]
        plot(d, x, y)
        plot_theory(data, x, eps)


    plt.xlabel(r'$\sin(\alpha_{min})$')
    plt.ylabel(r'$|V_\varepsilon|$')
    plt.xscale('log')
    plt.yscale('log') 
    plt.xlim((plt.xlim()[1], plt.xlim()[0])) 
    plt.tight_layout() 
    plt.legend()
    plt.show()

