#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=4

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# graph files
TRIANGULATION_GRAPH=$GRAPH_DIR/aegaeis/coastlines.graph

# output paths
OUTPUT_DIR=results/coastlines
QUERY_FILE=results/coastlines/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
if [ ! -f "$QUERY_FILE" ]; then
  make_queries "$UNREF_TRIANGULATION_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi

# ######################################### refined graph ########################################
# refined graph without points
if [ ! -d "$OUTPUT_DIR/ref" ]; then
  compute_ota_queries "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref/inf/" "$QUERY_FILE" inf

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref/$eps/" "$QUERY_FILE" "$eps" "--no-tree"
  done
fi
process_results "$OUTPUT_DIR/ref/" "$OUTPUT_DIR/results-ref.csv" aegaeis-ref

######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
