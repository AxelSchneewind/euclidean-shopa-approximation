#!/bin/python

import pandas as pd
import numpy as np

import sys
import math

dtypes = {
    'EPSILON' : float,
    'COST' : float,
    'NODE COUNT' : int,
    'EDGE COUNT' : int,
    'STORED NODE COUNT' : int,
    'STORED EDGE COUNT' : int
}

def load(file):
    data = pd.read_csv(file, dtype=dtypes)
    return data

def reference(data, row):
    fr = row['FROM']
    to = row['TO']
    matches = data.loc[(data['EPSILON'] == 0.0) & (data['FROM'] == fr) & (data['TO'] == to)]
    return matches

def add_optimal_cost(data):
    r = []
    for i, row in data.iterrows():
        ref = reference(data, row)
        if ref.shape[0] != 0:
            r = r + [ref['COST']]
        else:
            r = r + [math.inf]
    data['OPTIMAL COST'] = r
    return data

def add_ratios(data):
    r = []
    for i, row in data.iterrows():
        ref = reference(data, row)
        if ref.shape[0] != 0 and ref['COST'].iloc[0] != 0.0:
            value = float(row['COST'] / ref['COST'])
            r = r + [value]
        else:
            r = r + [math.inf]
    data['RATIO'] = r
    return data

def filter_epsilon(data, eps):
    return data[data['EPSILON'] == eps]

def exclude_epsilon(data, eps):
    return data[data['EPSILON'] != eps]

def filter(data):
    data = data.loc[(data['COST'] != math.inf) & (data['COST'] != math.nan)]
    has_reference = [reference(data, row).shape[0] != 0 for i, row in data.iterrows()]
    data = data.loc[has_reference]
    return data


def main():
    data = load(sys.argv[1])
    data = filter(data)

    data = add_optimal_cost(data)
    data = add_ratios(data)

    data_vis = filter_epsilon(data, 0.0)
    data_app = exclude_epsilon(data, 0.0)

    t = data['TIME']
    t = t[t != math.inf]

    print('mean ratio,max ratio,mean time,max time', sep='')
    print(data['RATIO'].mean(), ',', data['RATIO'].max(), ',', t.mean(), ',', t.max(), sep='')

if __name__ == "__main__":
    main()
