#!python


from matplotlib import pyplot as plt
import pandas as pd

import sys

if (len(sys.argv) < 2):
    print('filename required')
    exit(-1)

def read(filename):
    return pd.read_csv(filename)


data = read(sys.argv[1])
data['index'].hist(bins=100)

plt.show()

