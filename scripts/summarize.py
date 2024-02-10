#!/bin/python

import pandas as pd
import numpy as np

import sys

def load(file):
    data = pd.read_csv(file)
    return data

def reference(data, row):
    eps = row['EPSILON']
    fr = row['FROM']
    to = row['TO']
    return data.loc[(data['EPSILON'] == 0) & (data['FROM'] == fr) & (data['TO'] == to)].iloc[0]

def ratios(data, ref_data):
    r = []
    for i in data.index:
        row = data.iloc[i]
        r = r + [ row['COST'] / reference(ref_data, row)['COST'] ]
    return np.array(r)

def timings(data):
    t = []
    for i in data.index:
        t = t + [ data['TIME'].iloc[i] ]
    return np.array(t)


def main():
    data_app = load(sys.argv[1])
    data_vis = load(sys.argv[2])

    
    r = ratios(data_app, data_vis)
    t = timings(data_app)
    print('mean ratio,max ratio,mean time,max time', sep='')
    print(np.mean(r), ',', np.max(r), ',', np.mean(t), ',', np.max(t), sep='')

if __name__ == "__main__":
    main()
