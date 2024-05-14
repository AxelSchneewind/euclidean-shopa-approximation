#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=6

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=10000

# toggle A* here (on/off)
ASTAR=on

# graph files
VISIBILITY_GRAPH=$GRAPH_DIR/toy/toy.fmi
TRIANGULATION_GRAPH=$GRAPH_DIR/toy/toy.graph

# output paths
OUTPUT_DIR=results/toy
QUERY_FILE=results/toy/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
if [ ! -f "$QUERY_FILE" ]; then
	make_queries "$TRIANGULATION_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi



################################# refined graph ###############################
# without points
if [ ! -d "$OUTPUT_DIR/raw" ]; then
	compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw" "$QUERY_FILE" inf ""
fi
process_results "$OUTPUT_DIR/raw" "$OUTPUT_DIR/results-raw.csv" toy-raw


# with steiner points
if [ ! -d "$OUTPUT_DIR/approximate" ]; then
	EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02")
	for eps in "${EPSILONS[@]}"; do
	    compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate/$eps" "$QUERY_FILE" "$eps" ""
	done
fi
process_results "$OUTPUT_DIR/approximate" "$OUTPUT_DIR/results-approximate.csv" toy-approximate



######################################## visibility graph #######################################
exact solutions
if [ ! -d "$OUTPUT_DIR/exact" ]; then
	compute_single "$VISIBILITY_GRAPH" "$OUTPUT_DIR/exact" "$QUERY_FILE" 0.0 ""
fi
process_results "$OUTPUT_DIR/exact" "$OUTPUT_DIR/results-exact.csv" toy-exact



######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
