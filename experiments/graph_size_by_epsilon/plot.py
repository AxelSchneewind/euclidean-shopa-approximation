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
    label = '$|V_\\varepsilon|$ for {}'.format(data['graph'].iloc[0])

    x_values = data[x_column]
    y_values = data[y_column]
    plt.scatter(x_values, y_values, marker='x', label=label)

# make theoretical upper bound
def plot_theory(data, x_column, graph):
    label = 'rough estimate for $|V| + \\sum D(e)$ for {}'.format(graph)

    base_node_count = int(data[data['graph'] == graph]['stored node count'].iloc[0])
    base_edge_count = int(data[data['graph'] == graph]['stored edge count'].iloc[0])

    x_values = data[x_column].drop_duplicates().sort_values()
    r = [base_node_count + 3 * base_edge_count + base_edge_count * 10/eps * max(1, math.log2(2/eps)) for eps in x_values]

    plt.plot(x_values, r, label=label)


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print('expected arguments: path/to/data.csv x_column y_column')
        exit(-1)

    file = sys.argv[1]
    x = sys.argv[2]
    y = sys.argv[3]

    data = read(file)

    for graph in data['graph'].drop_duplicates():
        d = data[data['graph'] == graph]
        plot(d, x, y)
        plot_theory(d, x, graph)


    plt.xlabel(r'$\varepsilon$')
    plt.ylabel(r'$|V_\varepsilon|$')
    plt.xscale('log')
    plt.yscale('log') 
    plt.xlim((plt.xlim()[1], plt.xlim()[0])) 
    plt.tight_layout() 
    plt.legend()
    plt.show()

