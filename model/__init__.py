import pandas as pd
import numpy as np

dtypes = {
    'epsilon': float,
    'source': int,
    'target': int,
    'cost': float,
    'time': float,
    'tree size': int,
    'astar': int,
    'neighbor finding algorithm': int,
    'pruning': int,
    'source latitude': float,
    'source longitude': float,
    'target latitude': float,
    'target longitude': float,
    'node count': int,
    'edge count': int,
    'stored node count': int,
    'stored edge count': int,
    'memory usage graph': float,
    'memory usage final': float,
    'beeline distance': float,
    'queue pull count': int,
    'queue push count': int,
    'queue max size': int,
    'edges checked': int,
    'neighbors base node count': int,
    'neighbors base node neighbors count': int,
    'neighbors boundary node count': int,
    'neighbors boundary node neighbors count': int,
    'neighbors steiner point count': int,
    'neighbors steiner point neighbors count': int,
    'neighbors steiner point search iteration count': int,
    'graph': str,
    'benchmark': str
}

column_units = {
    'epsilon': '',
    'source': '',
    'target': '',
    'cost': '',
    'time': 'ms',
    'tree size': 'nodes',
    'astar': '',
    'neighbor finding algorithm': '',
    'pruning': '',
    'source latitude': 'º',
    'source longitude': 'º',
    'target latitude': 'º',
    'target longitude': 'º',
    'node count': '',
    'edge count': '',
    'stored node count': '',
    'stored edge count': '',
    'memory usage graph': 'MiB',
    'memory usage final': 'MiB',
    'beeline distance': '',
    'queue pull count': '',
    'queue push count': '',
    'queue max size': '',
    'edges checked': '',
    'neighbors base node count': '',
    'neighbors base node neighbors count': '',
    'neighbors boundary node count': '',
    'neighbors boundary node neighbors count': '',
    'neighbors steiner point count': '',
    'neighbors steiner point neighbors count': '',
    'neighbors steiner point search iteration count': '',
    'graph': '',
    'benchmark': '',
    'ratio': '',
    'optimal cost': ''
}

converters = {

}


def reference(data, row):
    fr = row['source']
    to = row['target']
    matches = data.loc[(data['epsilon'] == 0.0) & (data['source'] == fr) & (data['target'] == to)]
    return matches


def add_optimal_cost(data):
    r = []
    for i, row in data.iterrows():
        ref = reference(data, row)
        if len(ref['cost']) > 0:
            r = r + [ref['cost'].iloc[0].item() ]
        else:
            r = r + [ 0.0 ]
    data['optimal cost'] = np.array(r, dtype='float')
    return data


def add_ratios(data):
    data['ratio'] = (data['cost']/data['optimal cost']).to_numpy(dtype='float')
    return data


def load(files, columns=None):
    data = pd.DataFrame()
    for f in files:
        data = pd.concat([data, pd.read_csv(f, dtype=dtypes, usecols=columns, converters=converters)], ignore_index=True)

    if columns is None:
        columns = list(dtypes)
    data = data[columns]

    # derive new columns from existing ones
    data = add_optimal_cost(data)
    data = add_ratios(data)

    return data

