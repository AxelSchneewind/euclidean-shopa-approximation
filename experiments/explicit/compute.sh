#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=100

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# graph files
TRIANGULATION_REF_GRAPH=milos-ref.graph
TRIANGULATION_UNREF_GRAPH=milos.graph
EXPLICIT_REF_GRAPH=milos-ref.fmi
# EXPLICIT_GRAPH=pruned.fmi

# output paths
OUTPUT_DIR=results/aegaeis
QUERY_FILE=results/aegaeis/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
if [ ! -f "$QUERY_FILE" ]; then
	make_queries "$TRIANGULATION_UNREF_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi


############################ refined graph using triangle (Shewchuk) ############################
# refined graph with steiner points
if [ ! -d "$OUTPUT_DIR/approximate-ref" ]; then
	EPSILONS=("1.0" "0.5" "0.25") # "0.125" "0.0625" "0.03125" "0.015625")
	for eps in "${EPSILONS[@]}"; do
	    compute_single "$TRIANGULATION_REF_GRAPH" "$OUTPUT_DIR/approximate-ref" "$QUERY_FILE" "$eps" ""
	done
fi
process_results "$OUTPUT_DIR/approximate-ref" "$OUTPUT_DIR/results-approximate-ref.csv" aegaeis-approximate-ref



######################################## unrefined graph ########################################
# unrefined graph with steiner points
if [ ! -d "$OUTPUT_DIR/approximate-unref" ]; then
	EPSILONS=("1.0" "0.5" "0.25")
	for eps in "${EPSILONS[@]}"; do
	    compute_single "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/approximate-unref" "$QUERY_FILE" "$eps" ""
	done
fi
process_results "$OUTPUT_DIR/approximate-unref" "$OUTPUT_DIR/results-approximate-unref.csv" aegaeis-approximate-unref


######################################### explicit graph ########################################
# exact solutions
if [ ! -d "$OUTPUT_DIR/explicit-ref" ]; then
	compute_single "$EXPLICIT_GRAPH" "$OUTPUT_DIR/explicit-ref" "$QUERY_FILE" 0.0 ""
fi
process_results "$OUTPUT_DIR/explicit-ref" "$OUTPUT_DIR/results-explicit-ref.csv" aegaeis-explicit-ref



######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
