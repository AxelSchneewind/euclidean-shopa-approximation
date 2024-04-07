#!/bin/bash

PYTHONPATH=/opt/routing/lib
export PYTHONPATH

PLOT_FILE_TYPE=.pgf

# triangle graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t triangle -c ratio --fliers -o boxplots_ratio_triangle$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t triangle -c time --fliers -o boxplots_time_triangle$PLOT_FILE_TYPE

# ref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t ref -c ratio --fliers -o boxplots_ratio_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t ref -c time --fliers -o boxplots_time_ref$PLOT_FILE_TYPE

# unref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t unref -c ratio --fliers -o boxplots_ratio_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t unref -c time --fliers -o boxplots_time_ref$PLOT_FILE_TYPE
