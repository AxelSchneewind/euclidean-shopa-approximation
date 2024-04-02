#!python


from matplotlib import pyplot as plt
import pandas as pd

import sys

def read(filename):
    return pd.read_csv(filename)

if (len(sys.argv) < 2):
    print('filename required')
    exit(-1)

data = read(sys.argv[1])
# data['index'].hist(bins=100)
plt.plot(data['angle'], data['count'])
plt.xlabel('angle')
plt.ylabel('count')
plt.title('distribution of inner angles')
plt.show()

