#!/bin/python

import pandas as pd
from matplotlib import pyplot as plt
import matplotlib as mpl
import math


# read graph statistics
data = pd.read_csv('graph_sizes.csv', sep=',', comment='#')
data.sort_values('epsilon', inplace=True)

# 
node_count = data['stored node count'][0]
edge_count = data['stored edge count'][0]

# columns
xValues = data['epsilon']
nValues = data['node count']
eValues = data['edge count']

# reference
num_values = len(xValues)
coefficient = nValues[num_values - 1] * pow(xValues[num_values-1], 1)
# rValues = [ coefficient / pow(x, 1) for x in xValues ]
#other_coefficient = (nValues[num_values-1] * xValues[num_values-1] * -1 / math.log(xValues[num_values -1], 2))
other_coefficient = edge_count
otherRValues = [ (coefficient / x * -1 * math.log(x, 2)) + node_count if x < 1.0 else node_count for x in xValues ] 

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
# nodes.plot(xValues, rValues, label='in Theta(1/(ε^2.5))')
nodes.plot(xValues, otherRValues, label='in Theta(1/ε * log(1/ε))')

nodes.set_yscale('log')
edges.set_yscale('log')
nodes.set_xscale(mpl.scale.LogScale(nodes, base=2))
edges.set_xscale(mpl.scale.LogScale(edges, base=2))


nodes.set_xlabel('ε')
edges.set_xlabel('ε')

nodes.legend()
edges.legend()
plt.show()
