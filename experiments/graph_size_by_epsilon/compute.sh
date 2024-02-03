#!/bin/bash

# specify paths to graph files here
GRAPH_AEGS=/opt/routing/graphs/visibility_aegaeis/aegaeis-ref.graph
GRAPH_PATA=/opt/routing/graphs/pata/pata-ref.graph
GRAPH_MEDI=/opt/routing/graphs/medi/medi-ref.graph

# make empty results directory
mkdir -p results
rm -rf results/*

# 
compute () {
graph_name=$(basename "$1")
graph_name=${graph_name%.graph}

eps=8.0

echo -e -n "\rstep 0: subdivision_stats graph $1 $eps >> results/results_${graph_name}.csv"
subdivision_stats graph "$1" "$eps" --header > results/results_${graph_name}.csv

for i in {1..16};
do
  eps=$(bc -l <<< "$eps / 2")
  # compute graph sizes
  echo -e -n "\rstep $i: subdivision_stats graph $1 $eps >> results/results_${graph_name}.csv"
  subdivision_stats graph "$1" "$eps" >> results/results_${graph_name}.csv
done
echo -e "                                           \rdone"
}

compute $GRAPH_AEGS
compute $GRAPH_PATA
compute $GRAPH_MEDI
#

csvstack results/result_*.csv > results/results.csv

csvlook results/results.csv
