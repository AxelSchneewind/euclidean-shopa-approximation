#!/bin/bash

source utils.sh

# number of queries
NUM_QUERIES=6

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

#
VISIBILITY_GRAPH=/opt/routing/graphs/toy/toy-vis.fmi
TRIANGULATION_GRAPH=/opt/routing/graphs/toy/toy.graph

# output paths
OUTPUT_DIR=results/toy
QUERY_FILE=results/toy/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
make_queries "$UNREF_TRIANGULATION_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"

# refined graph
compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw" "$QUERY_FILE" inf
#EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02")
EPSILONS=("1.0" "0.5")
for eps in "${EPSILONS[@]}"; do
    compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate" "$QUERY_FILE" "$eps"
done

# unrefined graph
# compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/raw-unref" "$QUERY_FILE" inf
# # EPSILONS=("1.0" "0.5" "0.2" "0.1")
# EPSILONS=("1.0" "0.5")
# for eps in "${EPSILONS[@]}"; do
#     compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/approximate-unref" "$QUERY_FILE" "$eps"
# done

# exact solutions
compute_single "$VISIBILITY_GRAPH" "$OUTPUT_DIR/exact" "$QUERY_FILE" 0.0

process_results "$OUTPUT_DIR/raw" "$OUTPUT_DIR/results-raw.csv"
# process_results "$OUTPUT_DIR/raw-unref""$OUTPUT_DIR/results-raw-unref.csv"
process_results "$OUTPUT_DIR/approximate""$OUTPUT_DIR/results-approximate.csv"
# process_results "$OUTPUT_DIR/approximate-unref""$OUTPUT_DIR/results-approximate-ref.csv"
process_results "$OUTPUT_DIR/exact" "$OUTPUT_DIR/results-exact.csv"

process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
