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


# triangle graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t triangle -c time --fliers -o plots/boxplots_time_triangle$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results-triangle.csv --output-epsilon=summaries/triangle-epsilon.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count'
python summarize.py -f results/aegaeis/results-triangle.csv --output-queries=summaries/triangle-queries.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count'

# ref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t ref -c time --fliers -o plots/boxplots_time_ref$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results-ref.csv --output-epsilon=summaries/ref-epsilon.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count'
python summarize.py -f results/aegaeis/results-ref.csv --output-queries=summaries/ref-queries.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count'

# unref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t unref -c time --fliers -o plots/boxplots_time_ref$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results-unref.csv --output-epsilon=summaries/unref-epsilon.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count'
python summarize.py -f results/aegaeis/results-unref.csv --output-queries=summaries/unref-queries.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count'

