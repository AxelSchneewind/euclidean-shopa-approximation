#!/bin/bash

source utils.sh

# number of queries
NUM_QUERIES=100

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# graph files
VISIBILITY_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-vis.fmi
TRIANGULATION_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-ref.graph
TRIANGLE_TRIANGULATION_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-ref-new.graph
UNREF_TRIANGULATION_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-unref.graph

# output paths
OUTPUT_DIR=results/aegaeis
QUERY_FILE=results/aegaeis/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
if [ ! -f "$QUERY_FILE" ]; then
	make_queries "$UNREF_TRIANGULATION_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi



######################################### refined graph ########################################
# refined graph without points
if [ ! -d "$OUTPUT_DIR/raw" ]; then
	compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw" "$QUERY_FILE" inf
fi
process_results "$OUTPUT_DIR/raw" "$OUTPUT_DIR/results-raw.csv"

# refined graph with steiner points
if [ ! -d "$OUTPUT_DIR/approximate" ]; then
	EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02")
	for eps in "${EPSILONS[@]}"; do
	    compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate" "$QUERY_FILE" "$eps"
	done
fi
process_results "$OUTPUT_DIR/approximate" "$OUTPUT_DIR/results-approximate.csv"



############################ refined graph using triangle (Shewchuk) ############################
# refined graph (triangle) without points
if [ ! -d "$OUTPUT_DIR/raw-triangle" ]; then
	compute_single "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw-triangle" "$QUERY_FILE" inf
fi
process_results "$OUTPUT_DIR/raw-triangle" "$OUTPUT_DIR/results-raw-triangle.csv"


# refined graph (triangle) with steiner points
if [ ! -d "$OUTPUT_DIR/approximate-triangle" ]; then
	EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02")
	for eps in "${EPSILONS[@]}"; do
	    compute_single "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate-triangle" "$QUERY_FILE" "$eps"
	done
fi
process_results "$OUTPUT_DIR/approximate-triangle" "$OUTPUT_DIR/results-approximate-triangle.csv"



######################################## unrefined graph ########################################
# unrefined graph raw
if [ ! -d "$OUTPUT_DIR/raw-unref" ]; then
	compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw-unref" "$QUERY_FILE" inf
fi
process_results "$OUTPUT_DIR/raw-unref" "$OUTPUT_DIR/results-raw-unref.csv"


# unrefined graph with steiner points
if [ ! -d "$OUTPUT_DIR/approximate-unref" ]; then
	# EPSILONS=("1.0" "0.5" "0.2" "0.1")
	EPSILONS=("1.0" "0.5")
	for eps in "${EPSILONS[@]}"; do
	    compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate-unref" "$QUERY_FILE" "$eps"
	done
fi
process_results "$OUTPUT_DIR/approximate-unref" "$OUTPUT_DIR/results-approximate-unref.csv"



######################################## visibility graph #######################################
# exact solutions
if [ ! -d "$OUTPUT_DIR/exact" ]; then
	compute_single "$VISIBILITY_GRAPH" "$OUTPUT_DIR/exact" "$QUERY_FILE" 0.0
fi
process_results "$OUTPUT_DIR/exact" "$OUTPUT_DIR/results-exact.csv"



######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
