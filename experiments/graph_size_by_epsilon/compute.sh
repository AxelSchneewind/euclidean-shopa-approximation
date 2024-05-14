#!/bin/bash

source ../utils.sh

# specify paths to graph files here
GRAPH_AEGS_REF=$GRAPH_DIR/aegaeis/aegaeis-ref.graph
GRAPH_AEGS_TRIANGLE=$GRAPH_DIR/aegaeis/aegaeis-ref-new.graph
GRAPH_AEGS_UNREF=$GRAPH_DIR/aegaeis/aegaeis-unref.graph
GRAPH_MILOS_REF=../explicit/milos-ref.graph
GRAPH_MILOS_UNREF=../explicit/milos.graph
GRAPH_PATA=$GRAPH_DIR/pata/pata-ref.graph
GRAPH_MEDI=$GRAPH_DIR/medi/medi-ref.graph

# make empty results directory
mkdir -p results
rm -rf results/*.csv

# usage: compute path/to/graph-file.graph
compute () {
	if [[ ! -e "$1" ]]; then
		exit
	fi

	graph_name=$(basename "$1")
	graph_name=${graph_name%.graph}
	
	eps=8.0
	
	echo -e -n "\rstep 0: graph_stats --mode steiner_graph_size -g $1 -e $eps >> results/results_${graph_name}.csv"
	graph_stats --mode steiner_graph_size -g "$1" -e "$eps" > results/results_${graph_name}.csv
	
	for i in {1..40};
	do
	  # halve epsilon
	  eps=$(bc -l <<< "$eps / 2")
	  # compute graph sizes
	  echo -e -n "\rstep $i: graph_stats --mode graph -g $1 -e $eps >> results/results_${graph_name}.csv"
	  graph_stats --mode steiner_graph_size -g "$1" -e "$eps" --no-header >> results/results_${graph_name}.csv
	done
	echo -e "                                           \rdone"
}

compute $GRAPH_AEGS_REF
compute $GRAPH_AEGS_TRIANGLE
compute $GRAPH_AEGS_UNREF
compute $GRAPH_MILOS_REF
compute $GRAPH_MILOS_UNREF
compute $GRAPH_PATA
compute $GRAPH_MEDI

# combine results
csvstack results/results_*.csv > results/results.csv
csvlook results/results.csv
