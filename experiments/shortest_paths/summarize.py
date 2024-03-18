#!/bin/python

import pandas as pd
import numpy as np

import sys
import math
import argparse

dtypes = {
    'epsilon': float,
    'source': int,
    'target': int,
    'node count': int,
    'edge count': int,
    'stored node count': int,
    'stored edge count': int,
    'time': float,
    'cost': float,
    'tree size': int,
}

converters = {

}


def load(files):
    data = pd.DataFrame()
    for f in files:
        data = pd.concat([data, pd.read_csv(f, dtype=dtypes, usecols=list(dtypes), converters=converters)], ignore_index=True)
    return data[list(dtypes)]       # to reorder the columns


def reference(data, row):
    fr = row['source']
    to = row['target']
    matches = data.loc[(data['epsilon'] == 0.0) & (data['source'] == fr) & (data['target'] == to)]
    return matches


def add_optimal_cost(data):
    r = []
    for i, row in data.iterrows():
        ref = reference(data, row)
        r = r + [ref['cost'].item()]
    data['optimal cost'] = r
    return data


def add_ratios(data):
    data['ratio'] = (data['cost'] / data['optimal cost'])
    return data


def filter_epsilon(data, eps):
    return data[data['epsilon'] == eps]


def exclude_epsilon(data, eps):
    return data[data['epsilon'] != eps]


def filter(data):
    invalid_cost = data.loc[(data['cost'] == math.inf) | (data['cost'] == math.nan)]
    if len(invalid_cost) != 0:
        print('ignored by cost value: ', file=sys.stderr)
        print(invalid_cost, file=sys.stderr)
    data = data.loc[(data['cost'] != math.inf) & (data['cost'] != math.nan)]
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
    parser.add_argument('--column', '-c', default='time', help='the column to summarize')
    args = parser.parse_args()

    # load data and filter out unusable results
    data = load(args.file)
    data = filter(data)

    # derive new columns from existing ones
    data = add_optimal_cost(data)
    data = add_ratios(data)


    # by epsilon
    eps_file = args.output_epsilon if args.output_epsilon != 'stdout' else sys.stdout
    print('epsilon,count,mean,min,median,max,std,p1,p10,p25,p75,p90,p99', sep='', file=eps_file)
    for epsilon in data['epsilon'].unique():
        by_epsilon = filter_epsilon(data, epsilon)
        by_epsilon = by_epsilon[args.column]
        print(epsilon, by_epsilon.count(), by_epsilon.mean(), by_epsilon.min(), by_epsilon.median(), by_epsilon.max(), by_epsilon.std(),
              by_epsilon.quantile(0.01), by_epsilon.quantile(0.1), by_epsilon.quantile(0.25),
              by_epsilon.quantile(0.75), by_epsilon.quantile(0.9), by_epsilon.quantile(0.99),
              sep=',', file=eps_file)

    # info on each query
    q_file = args.output_queries if args.output_queries != 'stdout' else sys.stdout
    print('source,target,count,min,median,max,std,p1,p10,p25,p75,p90,p99', sep='', file=q_file)
    for s in data['source'].unique():
        for t in data[(data['source'] == s)]['target'].unique():
            by_query = data[(data['source'] == s) & (data['target'] == t)]
            by_query = by_query[args.column]
            print(s, t, by_query.count(), by_query.mean(), by_query.min(), by_query.median(), by_query.max(), by_query.std(),
                by_query.quantile(0.01), by_query.quantile(0.1), by_query.quantile(0.25),
                by_query.quantile(0.75), by_query.quantile(0.9), by_query.quantile(0.99),
                sep=',', file=q_file)

    #


if __name__ == "__main__":
    main()
