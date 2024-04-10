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
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=triangle -c ratio --fliers -o plots/boxplots_ratio_triangle$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=triangle -c time --fliers -o plots/boxplots_time_triangle$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results.csv -t=triangle --output-epsilon=summaries/triangle-epsilon.csv --column=ratio
python summarize.py -f results/aegaeis/results.csv -t=triangle --output-epsilon=summaries/triangle-epsilon.csv --column=time
python summarize.py -f results/aegaeis/results.csv -t=triangle --output-queries=summaries/triangle-queries.csv --column=ratio
python summarize.py -f results/aegaeis/results.csv -t=triangle --output-queries=summaries/triangle-queries.csv --column=time

# ref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=ref -c ratio --fliers -o plots/boxplots_ratio_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=ref -c time --fliers -o plots/boxplots_time_ref$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results.csv -t=ref --output-epsilon=summaries/ref-epsilon.csv --column=ratio
python summarize.py -f results/aegaeis/results.csv -t=ref --output-epsilon=summaries/ref-epsilon.csv --column=time
python summarize.py -f results/aegaeis/results.csv -t=ref --output-queries=summaries/ref-queries.csv --column=ratio
python summarize.py -f results/aegaeis/results.csv -t=ref --output-queries=summaries/ref-queries.csv --column=time

# unref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=unref -c=ratio --fliers -o plots/boxplots_ratio_unref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=unref -c=time --fliers -o plots/boxplots_time_unref$PLOT_FILE_TYPE

python summarize.py -f results/aegaeis/results.csv -t=unref --output-epsilon=summaries/unref-epsilon.csv --column=ratio
python summarize.py -f results/aegaeis/results.csv -t=unref --output-epsilon=summaries/unref-epsilon.csv --column=time
python summarize.py -f results/aegaeis/results.csv -t=unref --output-queries=summaries/unref-queries.csv --column=ratio
python summarize.py -f results/aegaeis/results.csv -t=unref --output-queries=summaries/unref-queries.csv --column=time

