#!/bin/python

import pandas as pd
from matplotlib import pyplot as plt
import matplotlib as mpl
import math
import sys


if __name__ == '__main__':
    # read graph statistics
    data = pd.read_csv('statistics.csv', sep=',', comment='#')
    data.sort_values('epsilon', inplace=True)

    if len(sys.argv) < 2 or not sys.argv[1] in data.columns:
        print('specify which column to plot (from ', data.columns, ')')
        exit(-1)
    column = sys.argv[1]

    # 
    node_count = data['stored node count'][0]
    edge_count = data['stored edge count'][0]
    
    # columns
    xValues = data['epsilon']
    yValues = data[column]
    
    # 
    (fig, ax) = plt.subplots(1,1)
    axis = ax
    
    base_color = 'green'
    
    coastlines_color = 'yellow'
    coastlines_node_count = 37470053
    coastlines_edge_count = 350600718
    
    
    axis.plot(xValues, yValues, label=column)
    
    axis.set_yscale('log')
    axis.set_xscale(mpl.scale.LogScale(axis, base=2))
    
    
    axis.set_xlabel('ε')
    
    axis.legend()
    plt.show()



    

