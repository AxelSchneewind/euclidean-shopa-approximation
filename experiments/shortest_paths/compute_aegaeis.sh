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
UNREF_TRIANGULATION_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-unref.graph

# output paths
OUTPUT_DIR=results/aegaeis
QUERY_FILE=results/aegaeis/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
make_queries "$UNREF_TRIANGULATION_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"

# refined graph
compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw" "$QUERY_FILE" inf
EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02")
for eps in "${EPSILONS[@]}"; do
    compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate" "$QUERY_FILE" "$eps"
done

# unrefined graph
compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw-unref" "$QUERY_FILE" inf
# EPSILONS=("1.0" "0.5" "0.2" "0.1")
EPSILONS=("1.0" "0.5")
for eps in "${EPSILONS[@]}"; do
    compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate-unref" "$QUERY_FILE" "$eps"
done

# exact solutions
# compute_single "$VISIBILITY_GRAPH" "$OUTPUT_DIR/exact" "$QUERY_FILE" 0.0

# process results for each case
process_results "$OUTPUT_DIR/raw" "$OUTPUT_DIR/results-raw.csv"
process_results "$OUTPUT_DIR/raw-unref" "$OUTPUT_DIR/results-raw-unref.csv"
process_results "$OUTPUT_DIR/approximate" "$OUTPUT_DIR/results-approximate.csv"
process_results "$OUTPUT_DIR/approximate-unref" "$OUTPUT_DIR/results-approximate-unref.csv"
# process_results "$OUTPUT_DIR/exact" "$OUTPUT_DIR/results-exact.csv"

# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
