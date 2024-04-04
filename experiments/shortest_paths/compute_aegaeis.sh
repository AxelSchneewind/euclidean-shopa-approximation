#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=106

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# graph files
GRAPH_DIR=/opt/routing/graphs
VISIBILITY_GRAPH=$GRAPH_DIR/aegaeis/aegaeis-vis.fmi
TRIANGULATION_GRAPH=$GRAPH_DIR/aegaeis/aegaeis-ref.graph
TRIANGLE_TRIANGULATION_GRAPH=$GRAPH_DIR/aegaeis/aegaeis-ref-new.graph
UNREF_TRIANGULATION_GRAPH=$GRAPH_DIR/aegaeis/aegaeis-unref.graph

# output paths
OUTPUT_DIR=results/aegaeis
QUERY_FILE=results/aegaeis/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
if [ ! -f "$QUERY_FILE" ]; then
  make_queries "$UNREF_TRIANGULATION_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi

# blacklist queries
QUERY_BLACKLIST=('140693,97656' '142694,10431' '97597,15167' '108011,97989' '88260,208670' '90928,97858')
for q in "${QUERY_BLACKLIST[@]}"; do
  sed -e "s/$q//g" -i "$QUERY_FILE"
done
sed -e 's/,,*/,/g' -i "$QUERY_FILE"

# ######################################### refined graph ########################################
# refined graph without points
if [ ! -d "$OUTPUT_DIR/ref" ]; then
  compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref" "$QUERY_FILE" inf

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02")
  for eps in "${EPSILONS[@]}"; do
    compute_single "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref/$eps" "$QUERY_FILE" "$eps"
  done
fi
process_results "$OUTPUT_DIR/ref" "$OUTPUT_DIR/results-ref.csv" aegaeis-ref

############################ refined graph using triangle (Shewchuk) ############################
# refined graph without points
if [ ! -d "$OUTPUT_DIR/triangle" ]; then
  compute_single "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle" "$QUERY_FILE" inf

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25" "0.125" "0.0625" "0.03125" "0.015625")
  for eps in "${EPSILONS[@]}"; do
    compute_single "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle/$eps" "$QUERY_FILE" "$eps"
  done
fi
process_results "$OUTPUT_DIR/triangle" "$OUTPUT_DIR/results-triangle.csv" aegaeis-triangle

######################################## unrefined graph ########################################
# unrefined graph raw
if [ ! -d "$OUTPUT_DIR/unref" ]; then
  compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref" "$QUERY_FILE" inf

  # unrefined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25")
  for eps in "${EPSILONS[@]}"; do
    compute_single "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref/$eps" "$QUERY_FILE" "$eps"
  done
fi
process_results "$OUTPUT_DIR/unref" "$OUTPUT_DIR/results-unref.csv" aegaeis-unref

######################################## visibility graph #######################################
# exact solutions
if [ ! -d "$OUTPUT_DIR/exact" ]; then
  compute_single "$VISIBILITY_GRAPH" "$OUTPUT_DIR/exact" "$QUERY_FILE" 0.0
fi
process_results "$OUTPUT_DIR/exact" "$OUTPUT_DIR/results-exact.csv" aegaeis-exact

######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
