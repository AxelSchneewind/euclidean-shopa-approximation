#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=108

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# graph files
GRAPH_DIR=/opt/routing/graphs
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
QUERY_BLACKLIST=('140693,97656' '142694,10431' '97597,15167' '108011,97989' '88260,208670' '90928,97858' '18617,60831' '187782,118306')
for q in "${QUERY_BLACKLIST[@]}"; do
  sed -e "s/$q//g" -i "$QUERY_FILE"
done
sed -e 's/,,*/,/g' -i "$QUERY_FILE"

############################ refined graph using triangle (Shewchuk) ############################
# refined graph (triangle) with steiner points
EPSILONS=("1.0" "0.5" "0.2") # "0.1" "0.05" "0.02")
if [ ! -d "$OUTPUT_DIR/ref-param" ]; then
  for eps in "${EPSILONS[@]}"; do
    compute_shopa_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref-param/$eps/" "$QUERY_FILE" "$eps" "--neighbor-finding=param"
  done
fi
if [ ! -d "$OUTPUT_DIR/ref-trigon" ]; then
  for eps in "${EPSILONS[@]}"; do
    compute_shopa_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref-trigon/$eps/" "$QUERY_FILE" "$eps" "--neighbor-finding=trig"
  done
fi
if [ ! -d "$OUTPUT_DIR/ref-binary" ]; then
  for eps in "${EPSILONS[@]}"; do
    compute_shopa_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref-binary/$eps/" "$QUERY_FILE" "$eps" "--neighbor-finding=binary"
  done
fi
process_results "$OUTPUT_DIR/ref-param" "$OUTPUT_DIR/results-ref-param.csv" param
process_results "$OUTPUT_DIR/ref-trigon" "$OUTPUT_DIR/results-ref-trigon.csv" trig
process_results "$OUTPUT_DIR/ref-binary" "$OUTPUT_DIR/results-ref-binary.csv" binary

# ######################################## unrefined graph ########################################
# # unrefined graph with steiner points
# if [ ! -d "$OUTPUT_DIR/unref-binary" ]; then
# 	# EPSILONS=("1.0" "0.5" "0.2" "0.1")
# 	EPSILONS=("1.0" "0.5")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_shopa_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-linear" "$QUERY_FILE" "$eps" --neighbor-finding=linear
# 	    compute_shopa_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-trigon" "$QUERY_FILE" "$eps" --neighbor-finding=trigonometry
# 	    compute_shopa_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-binary" "$QUERY_FILE" "$eps" --neighbor-finding=binary
# 	done
# fi
# process_results "$OUTPUT_DIR/unref-linear" "$OUTPUT_DIR/results-unref-linear.csv" linear
# process_results "$OUTPUT_DIR/unref-trigon" "$OUTPUT_DIR/results-unref-trigon.csv" trigonometry
# process_results "$OUTPUT_DIR/unref-binary" "$OUTPUT_DIR/results-unref-binary.csv" binary

######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
