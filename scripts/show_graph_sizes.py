#!/bin/python

import pandas as pd
from matplotlib import pyplot as plt


# read graph statistics
data = pd.read_csv('statistics.csv', sep=',', comment='#')
data.sort_values('epsilon', inplace=True)

# columns
xValues = data['epsilon']
nValues = data['node count']
eValues = data['edge count']

# 
(fig, ax) = plt.subplots(1,2)
nodes = ax[0]
edges = ax[1]

base_color = 'green'

coastlines_color = 'yellow'
coastlines_node_count = 37470053
coastlines_edge_count = 350600718


(xmin, xmax) = (min(data['epsilon']), max(data['epsilon']))
nodes.hlines(data['stored node count'][0], xmin, xmax, label='base node count', color=base_color)
edges.hlines(data['stored edge count'][0], xmin, xmax, label='base edge count', color=base_color)

nodes.hlines(coastlines_node_count, xmin, xmax, label='coastlines base node count', color=coastlines_color)
edges.hlines(coastlines_edge_count, xmin, xmax, label='coastlines base edge count', color=coastlines_color)

nodes.plot(xValues, nValues, label='number of nodes')
edges.plot(xValues, eValues, label='number of edges (undirected)')

nodes.set_yscale('log')
edges.set_yscale('log')
nodes.set_xscale('log')
edges.set_xscale('log')


nodes.set_xlabel('ε')
edges.set_xlabel('ε')

nodes.legend()
edges.legend()
plt.show()
