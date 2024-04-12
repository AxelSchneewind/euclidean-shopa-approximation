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
EXPLICIT_REF_10_GRAPH=milos-ref-10.fmi
EXPLICIT_REF_05_GRAPH=milos-ref-05.fmi
EXPLICIT_REF_025_GRAPH=milos-ref-025.fmi
# EXPLICIT_GRAPH=pruned.fmi

# output paths
OUTPUT_DIR=results/milos
QUERY_FILE=results/milos/queries.txt

mkdir -p "$OUTPUT_DIR"

# make queries
if [ ! -f "$QUERY_FILE" ]; then
	make_queries "$TRIANGULATION_UNREF_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi


############################ refined graph using triangle (Shewchuk) ############################
# refined graph with steiner points
if [ ! -d "$OUTPUT_DIR/implicit-ref" ]; then
	EPSILONS=("1.0" "0.5" "0.25") # "0.125" "0.0625" "0.03125" "0.015625")
	for eps in "${EPSILONS[@]}"; do
	    compute_ota_queries "$TRIANGULATION_REF_GRAPH" "$OUTPUT_DIR/implicit-ref/$eps" "$QUERY_FILE" "$eps" ""
	done
fi
process_results "$OUTPUT_DIR/implicit-ref" "$OUTPUT_DIR/results-implicit-ref.csv" milos-implicit-ref



######################################## unrefined graph ########################################
# unrefined graph with steiner points
# if [ ! -d "$OUTPUT_DIR/implicit-unref" ]; then
# 	EPSILONS=("1.0" "0.5" "0.25")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_ota_queries "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/implicit-unref/$eps" "$QUERY_FILE" "$eps" ""
# 	done
# fi
# process_results "$OUTPUT_DIR/implicit-unref" "$OUTPUT_DIR/results-implicit-unref.csv" milos-implicit-unref


######################################### explicit graph ########################################
# exact solutions
if [ ! -d "$OUTPUT_DIR/explicit-ref-05" ]; then
	compute_ota_queries "$EXPLICIT_REF_10_GRAPH" "$OUTPUT_DIR/explicit-ref/1.0" "$QUERY_FILE" 1.0 ""
	compute_ota_queries "$EXPLICIT_REF_05_GRAPH" "$OUTPUT_DIR/explicit-ref/0.5" "$QUERY_FILE" 0.5 ""
	compute_ota_queries "$EXPLICIT_REF_025_GRAPH" "$OUTPUT_DIR/explicit-ref/0.25" "$QUERY_FILE" 0.25 ""
fi
process_results "$OUTPUT_DIR/explicit-ref/1.0" "$OUTPUT_DIR/results-explicit-ref-10.csv" milos-explicit-ref-10
process_results "$OUTPUT_DIR/explicit-ref/0.5" "$OUTPUT_DIR/results-explicit-ref-05.csv" milos-explicit-ref-05
process_results "$OUTPUT_DIR/explicit-ref/0.5" "$OUTPUT_DIR/results-explicit-ref-025.csv" milos-explicit-ref-025



######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
