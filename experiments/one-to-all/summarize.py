#!/bin/python

import model as bench

import pandas as pd
import numpy as np

import sys
import math
import argparse

columns = {
    'epsilon',
    'source',
    'target',
    'source latitude',
    'source longitude',
    'target latitude',
    'target longitude',
    'node count',
    'edge count',
    'stored node count',
    'stored edge count',
    'time',
    'cost',
    'tree size'
}

def filter_epsilon(data, eps):
    return data[data['epsilon'] == eps]


def exclude_epsilon(data, eps):
    return data[data['epsilon'] != eps]


def filter(data):
   # check that exact value exists
    # has_reference = np.array([reference(data, row).shape[0] != 0 for i, row in data.iterrows()], dtype='bool')
    # if len(has_reference[(has_reference == False)]) != 0:
    #     print('exact value missing for some queries', file=sys.stderr)
    #     print(data.loc[has_reference == False], file=sys.stderr)
    # data = data.loc[has_reference]

    # check that coordinates of source and target coordinates match
    coords_match = np.array([
        ((bench.reference(data, row)['source latitude'] == row['source latitude'])
        & (bench.reference(data, row)['source longitude'] == row['source longitude'])).all() for i,row in data.iterrows() ], dtype='bool')
    if len(coords_match[(coords_match == False)]) != 0:
        print('mismatch in source/target coordinates', file=sys.stderr)
        print(data.loc[coords_match == False], file=sys.stderr)
    data = data.loc[coords_match]

    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', required=True, action='append', help='path to the input file')
    parser.add_argument('--output-epsilon', '-e', default='stdout', help='path to the output file for statistics per epsilon')
    parser.add_argument('--output-queries', '-q', default='stdout', help='path to the output file for statistics per query')
    parser.add_argument('--output-worst-quality-queries', '-s', default='stdout', help='path to the output file for a list of the queries with worst quality')
    parser.add_argument('--columns', '-c', default='time', help='the columns to summarize (comma separated list)')
    parser.add_argument('--graph-type', '-t', default='triangle')
    args = parser.parse_args()

    # list of columns can be given
    columns = args.columns.split(',')

    # load data and filter out unusable results
    data = bench.load(args.file)
    data = filter(data)

    # compare given graph type to exact values
    graph_type = '-' + args.graph_type
    data = data.loc[(data['benchmark'].str.contains(graph_type)) | (data['benchmark'].str.contains('-exact'))]
    print('using benchmarks: ', data['benchmark'].unique())

    # list suspicious queries
    sus = data.loc[data['ratio'] > 1.05][['source', 'target', 'epsilon', 'cost', 'ratio']]
    sus.sort_values(by='ratio', inplace=True, ascending=False)
    print('some results are suspiciously bad: ', sus)

    # by epsilon
    eps_file = open(args.output_epsilon, mode='a') if args.output_epsilon != 'stdout' else sys.stdout
    print('column,epsilon,count,mean,min,median,max,std,p1,p10,p25,p75,p90,p99', sep='', file=eps_file)
    for column in columns:
        for epsilon in data['epsilon'].unique():
            by_epsilon = filter_epsilon(data, epsilon)
            by_epsilon = by_epsilon[column]
            print(column, epsilon, by_epsilon.count(), by_epsilon.mean(), by_epsilon.min(), by_epsilon.median(), by_epsilon.max(), by_epsilon.std(),
                  by_epsilon.quantile(0.01), by_epsilon.quantile(0.1), by_epsilon.quantile(0.25),
                  by_epsilon.quantile(0.75), by_epsilon.quantile(0.9), by_epsilon.quantile(0.99),
                  sep=',', file=eps_file)

    # info on each query
    q_file = open(args.output_queries, mode='a') if args.output_queries != 'stdout' else sys.stdout
    print('column,source,target,count,min,median,max,std,p1,p10,p25,p75,p90,p99', sep='', file=q_file)
    for column in columns:
        for s in data['source'].unique():
            for t in data[(data['source'] == s)]['target'].unique():
                by_query = data[(data['source'] == s) & (data['target'] == t)]
                by_query = by_query[column]
                print(column, s, t, by_query.count(), by_query.mean(), by_query.min(), by_query.median(), by_query.max(), by_query.std(),
                    by_query.quantile(0.01), by_query.quantile(0.1), by_query.quantile(0.25),
                    by_query.quantile(0.75), by_query.quantile(0.9), by_query.quantile(0.99),
                    sep=',', file=q_file)


if __name__ == "__main__":
    main()
