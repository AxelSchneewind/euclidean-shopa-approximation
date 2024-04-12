#!python


from matplotlib import pyplot as plt
import pandas as pd

import sys

def read(filename):
    return pd.read_csv(filename)

# plot histogram given by count y per value x
def hist(x, y):
    plt.plot(x,y)

def main():
    if (len(sys.argv) < 2):
        print('filename required')
        exit(-1)
    
    data = read(sys.argv[1])

    # compute cumulative distribution
    current_sum = 0
    hist_cumulative = []
    for count in data['count']:
        current_sum = current_sum + count
        hist_cumulative = hist_cumulative + [current_sum]
    data['count cumulative'] = hist_cumulative

    # plot histogram
    hist(data['angle'], data['count'])
    plt.xlabel('angle')
    plt.ylabel('count')
    plt.title('distribution of inner angles')
    plt.show()

    # plot cumulative histogram 
    hist(data['angle'], data['count cumulative'])
    plt.xlabel('angle')
    plt.ylabel('count')
    plt.title('cumulative distribution of inner angles')
    plt.show()

if __name__ == '__main__':
    main()
