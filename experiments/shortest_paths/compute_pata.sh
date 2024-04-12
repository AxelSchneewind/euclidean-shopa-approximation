#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=107

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# graph files
VISIBILITY_GRAPH=$GRAPH_DIR/pata/pata-vis.fmi
TRIANGULATION_GRAPH=$GRAPH_DIR/pata/pata-ref.graph

# output paths
OUTPUT_DIR=results/pata
QUERY_FILE=results/pata/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
if [ ! -f "$QUERY_FILE" ]; then
  make_queries "$VISIBILITY_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi

# blacklist queries
# QUERY_BLACKLIST=('140693,97656' '142694,10431' '97597,15167' '108011,97989' '88260,208670' '90928,97858' '18617,60831')
# for q in "${QUERY_BLACKLIST[@]}"; do
#   sed -e "s/$q//g" -i "$QUERY_FILE"
# done
# sed -e 's/,,*/,/g' -i "$QUERY_FILE"

# ######################################### refined graph ########################################
# refined graph without points
if [ ! -d "$OUTPUT_DIR/ref" ]; then
  compute_shopa_queries "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref/inf/" "$QUERY_FILE" inf

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02")
  for eps in "${EPSILONS[@]}"; do
    compute_shopa_queries "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref/$eps/" "$QUERY_FILE" "$eps"
  done
fi
process_results "$OUTPUT_DIR/ref" "$OUTPUT_DIR/results-ref.csv" pata-ref

######################################## visibility graph #######################################
# exact solutions
if [ ! -d "$OUTPUT_DIR/vis" ]; then
  compute_shopa_queries "$VISIBILITY_GRAPH" "$OUTPUT_DIR/vis/0.0/" "$QUERY_FILE" 0.0
fi
process_results "$OUTPUT_DIR/vis" "$OUTPUT_DIR/results-exact.csv" pata-exact

######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
