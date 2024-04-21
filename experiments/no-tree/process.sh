#!/bin/bash

PYTHONPATH=/opt/routing/lib
export PYTHONPATH

PLOT_FILE_TYPE=.pgf

# create empty directory for summaries
rm -rf summaries
mkdir summaries

# create empty directory for plots
rm -rf plots
mkdir plots

# coastlines graph
python plot_over_epsilon.py -f results/coastlines/results.csv -t=triangle -c=time --fliers -o plots/boxplots_time_triangle$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/coastlines/results.csv -t=triangle -c='memory usage final' --fliers -o plots/boxplots_mem_triangle$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/coastlines/results.csv -t=triangle -c='tree size' --fliers -o plots/boxplots_tree_triangle$PLOT_FILE_TYPE
python summarize.py -f results/coastlines/results.csv -t=triangle --output-epsilon=summaries/triangle-epsilon.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/coastlines/results.csv -t=triangle --output-queries=summaries/triangle-queries.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/coastlines/results.csv -t=triangle --output-benchmark=summaries/triangle-benchmarks.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'

