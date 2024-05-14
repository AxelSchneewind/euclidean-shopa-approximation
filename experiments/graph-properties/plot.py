#!python


from matplotlib import pyplot as plt
import pandas as pd

import re
import sys

def read(filename):
    return pd.read_csv(filename)

# plot histogram given by count y per value x
def hist(x, y, ax):
    ax.bar(x,y, width=x[1]-x[0])

def hist_cumul(x, y, ax):
    ax.plot(x,y)
    # ax.scatter(x,y)

def main():
    if (len(sys.argv) < 2):
        print('filename required')
        exit(-1)

    if len(sys.argv) > 2:
        print('plotting multiple')


    plot_count = len(sys.argv) - 1
    fig, ax = plt.subplots(1, plot_count, sharex=True)
    fig.set_size_inches(9, 3.5)


    data = read(sys.argv[1])
    # fig.supxlabel(data.columns[0])
    # fig.supylabel('count')
    
    # plot
    for index, file in enumerate(sys.argv[1:]):
        data = read(file)
        axis = ax[index]
        title = re.sub(r'/[^/]*.csv', '', file)
        title = re.sub(r'results/', '', title)
        title = re.sub(r'/', '-', title)
        axis.set_title(title)

        # plot histogram
        hist(data.iloc[:, 0], data['count'], axis)

    # plt.show()
    plt.tight_layout()

    # plt.cla()
    fig, ax = plt.subplots(1, plot_count, sharex=True)
    fig.set_size_inches(9, 3.5)

    # fig.supxlabel(data.columns[0])
    # fig.supylabel('count')

    for index, file in enumerate(sys.argv[1:]):
        data = read(file)
        axis = ax[index]
        title = re.sub(r'/[^/]*.csv', '', file)
        title = re.sub(r'results/', '', title)
        title = re.sub(r'/', '-', title)
        axis.set_title(title)

        # compute cumulative distribution
        current_sum = 0
        hist_cumulative = []
        for count in data['count']:
            current_sum = current_sum + count
            hist_cumulative = hist_cumulative + [current_sum]
        data['count cumulative'] = hist_cumulative

        # plot cumulative histogram 
        hist_cumul(data.iloc[:, 0], data['count cumulative'], ax[index])

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
