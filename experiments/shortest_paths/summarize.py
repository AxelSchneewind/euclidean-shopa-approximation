#!/bin/python

import pandas as pd
import numpy as np

import sys
import math
import argparse

dtypes = {
    'EPSILON': float,
    'FROM': int,
    'TO': int,
    'NODE COUNT': int,
    'EDGE COUNT': int,
    'STORED NODE COUNT': int,
    'STORED EDGE COUNT': int,
    'TIME': float,
    'COST': float,
    'TREE SIZE': int,
}

converters = {

}


def load(files):
    data = pd.DataFrame()
    for f in files:
        data = pd.concat([data, pd.read_csv(f, dtype=dtypes, usecols=list(dtypes), converters=converters)], ignore_index=True)
    return data[list(dtypes)]       # to reorder the columns


def reference(data, row):
    fr = row['FROM']
    to = row['TO']
    matches = data.loc[(data['EPSILON'] == 0.0) & (data['FROM'] == fr) & (data['TO'] == to)]
    return matches


def add_optimal_cost(data):
    r = []
    for i, row in data.iterrows():
        ref = reference(data, row)
        r = r + [ref['COST'].item()]
    data['OPTIMAL COST'] = r
    return data


def add_ratios(data):
    data['RATIO'] = (data['COST'] / data['OPTIMAL COST'])
    return data


def filter_epsilon(data, eps):
    return data[data['EPSILON'] == eps]


def exclude_epsilon(data, eps):
    return data[data['EPSILON'] != eps]


def filter(data):
    invalid_cost = data.loc[(data['COST'] == math.inf) | (data['COST'] == math.nan)]
    if len(invalid_cost) != 0:
        print('ignored by cost value: ', file=sys.stderr)
        print(invalid_cost, file=sys.stderr)
    data = data.loc[(data['COST'] != math.inf) & (data['COST'] != math.nan)]
    has_reference = np.array([reference(data, row).shape[0] != 0 for i, row in data.iterrows()], dtype='bool')
    if len(has_reference[(has_reference == False)]) != 0:
        print('exact value missing for some queries', file=sys.stderr)
        print(has_reference[has_reference == False], file=sys.stderr)
    data = data.loc[has_reference]
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True, action='append', help='path to the input file')
    parser.add_argument('--output-epsilon', '-e', default='stdout', help='path to the output file for statistics per epsilon')
    parser.add_argument('--output-queries', '-q', default='stdout', help='path to the output file for statistics per query')
    args = parser.parse_args()

    # load data and filter out unusable results
    data = load(args.file)
    data = filter(data)

    # derive new columns from existing ones
    data = add_optimal_cost(data)
    data = add_ratios(data)


    # by epsilon
    eps_file = args.output_epsilon if args.output_epsilon != 'stdout' else sys.stdout
    print('epsilon,mean ratio,max ratio,mean time,max time', sep='', file=eps_file)
    for epsilon in data['EPSILON'].unique():
        by_epsilon = filter_epsilon(data, epsilon)
        print(epsilon, by_epsilon['RATIO'].mean(), by_epsilon['RATIO'].max(), by_epsilon['TIME'].mean(), by_epsilon['TIME'].max(), sep=',', file=eps_file)

    # info on each query
    q_file = args.output_queries if args.output_queries != 'stdout' else sys.stdout
    print('from,to,mean ratio,max ratio,mean time,max time', sep='', file=q_file)
    for s in data['FROM'].unique():
        for t in data[(data['FROM'] == s)]['TO'].unique():
            by_query = data[(data['FROM'] == s) & (data['TO'] == t)]
            print(s,t, by_query['RATIO'].mean(), by_query['RATIO'].max(), by_query['TIME'].mean(), by_query['TIME'].max(), sep=',', file=q_file)

    #


if __name__ == "__main__":
    main()
