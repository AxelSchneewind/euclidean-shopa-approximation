import pandas as pd

dtypes = {
    'epsilon': float,
    'source': int,
    'target': int,
    'cost': float,
    'time': float,
    'tree size': int,
    'astar': int,
    'neighbor finding algorithm': int,
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
        r = r + [ref['cost'].item()]
    data['optimal cost'] = numpy.array(r, dtype='float')
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

