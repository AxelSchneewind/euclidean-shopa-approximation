#!/bin/bash

PYTHONPATH=/opt/routing/lib
export PYTHONPATH

PLOT_FILE_TYPE=.png

# AEGAEIS_RESULTS=results-no-astar/aegaeis/results.csv
# PLOT_DIR=plots-no-astar/
# SUM_DIR=summaries-no-astar/
AEGAEIS_RESULTS=results/aegaeis/results.csv
PLOT_DIR=plots/
SUM_DIR=summaries/

mkdir $PLOT_DIR
mkdir $SUM_DIR

# triangle graph
python plot_over_epsilon.py -f $AEGAEIS_RESULTS -t=triangle -c ratio --fliers -o $PLOT_DIR/boxplots_ratio_triangle$PLOT_FILE_TYPE
python plot_over_epsilon.py -f $AEGAEIS_RESULTS -t=triangle -c time -o $PLOT_DIR/boxplots_time_triangle$PLOT_FILE_TYPE
python summarize.py -f $AEGAEIS_RESULTS -t=triangle --output-epsilon=$SUM_DIR/triangle-epsilon.csv --column=ratio
python summarize.py -f $AEGAEIS_RESULTS -t=triangle --output-epsilon=$SUM_DIR/triangle-epsilon.csv --column=time
python summarize.py -f $AEGAEIS_RESULTS -t=triangle --output-queries=$SUM_DIR/triangle-queries.csv --column=ratio
python summarize.py -f $AEGAEIS_RESULTS -t=triangle --output-queries=$SUM_DIR/triangle-queries.csv --column=time

# ref graph
python plot_over_epsilon.py -f $AEGAEIS_RESULTS -t=ref -c ratio --fliers -o $PLOT_DIR/boxplots_ratio_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f $AEGAEIS_RESULTS -t=ref -c time -o $PLOT_DIR/boxplots_time_ref$PLOT_FILE_TYPE
python summarize.py -f $AEGAEIS_RESULTS -t=ref --output-epsilon=$SUM_DIR/ref-epsilon.csv --column=ratio
python summarize.py -f $AEGAEIS_RESULTS -t=ref --output-epsilon=$SUM_DIR/ref-epsilon.csv --column=time
python summarize.py -f $AEGAEIS_RESULTS -t=ref --output-queries=$SUM_DIR/ref-queries.csv --column=ratio
python summarize.py -f $AEGAEIS_RESULTS -t=ref --output-queries=$SUM_DIR/ref-queries.csv --column=time

# unref graph
python plot_over_epsilon.py -f $AEGAEIS_RESULTS -t=unref -c=ratio --fliers -o $PLOT_DIR/boxplots_ratio_unref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f $AEGAEIS_RESULTS -t=unref -c=time -o $PLOT_DIR/boxplots_time_unref$PLOT_FILE_TYPE

python summarize.py -f $AEGAEIS_RESULTS -t=unref --output-epsilon=$SUM_DIR/unref-epsilon.csv --column=ratio
python summarize.py -f $AEGAEIS_RESULTS -t=unref --output-epsilon=$SUM_DIR/unref-epsilon.csv --column=time
python summarize.py -f $AEGAEIS_RESULTS -t=unref --output-queries=$SUM_DIR/unref-queries.csv --column=ratio
python summarize.py -f $AEGAEIS_RESULTS -t=unref --output-queries=$SUM_DIR/unref-queries.csv --column=time

