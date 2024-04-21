#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=50

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
	make_ota_queries "$TRIANGULATION_UNREF_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi


############################ refined graph using triangle (Shewchuk) ############################
# no pruning
if [ ! -d "$OUTPUT_DIR/implicit-ref-unpruned" ]; then
	EPSILONS=("1.0" "0.5" "0.25")
	for eps in "${EPSILONS[@]}"; do
	    compute_ota_queries "$TRIANGULATION_REF_GRAPH" "$OUTPUT_DIR/implicit-ref-unpruned/$eps" "$QUERY_FILE" "$eps" "--pruning=none"
	done
fi
process_results "$OUTPUT_DIR/implicit-ref-unpruned" "$OUTPUT_DIR/results-implicit-ref-unpruned.csv" milos-implicit-ref-unpruned

# default pruning
if [ ! -d "$OUTPUT_DIR/implicit-ref-pruned" ]; then
	EPSILONS=("1.0" "0.5" "0.25")
	for eps in "${EPSILONS[@]}"; do
	    compute_ota_queries "$TRIANGULATION_REF_GRAPH" "$OUTPUT_DIR/implicit-ref-pruned/$eps" "$QUERY_FILE" "$eps" "--pruning=prune"
	done
fi
process_results "$OUTPUT_DIR/implicit-ref-pruned" "$OUTPUT_DIR/results-implicit-ref-pruned.csv" milos-implicit-ref-pruned

# pruned by minimal bending angle
if [ ! -d "$OUTPUT_DIR/implicit-ref-pruned-min-angle" ]; then
	EPSILONS=("1.0" "0.5" "0.25")
	for eps in "${EPSILONS[@]}"; do
	    compute_ota_queries "$TRIANGULATION_REF_GRAPH" "$OUTPUT_DIR/implicit-ref-pruned-min-angle/$eps" "$QUERY_FILE" "$eps" "--pruning=prune-min-angle"
	done
fi
process_results "$OUTPUT_DIR/implicit-ref-pruned-min-angle" "$OUTPUT_DIR/results-implicit-ref-pruned-min-angle.csv" milos-implicit-ref-pruned-min-angle

# default pruning with explicit coordinate storage
if [ ! -d "$OUTPUT_DIR/implicit-ref-pruned-exp" ]; then
	EPSILONS=("1.0" "0.5" "0.25")
	for eps in "${EPSILONS[@]}"; do
	    compute_ota_queries "$TRIANGULATION_REF_GRAPH" "$OUTPUT_DIR/implicit-ref-pruned-exp/$eps" "$QUERY_FILE" "$eps" "--pruning=prune --coords-explicit"
	done
fi
process_results "$OUTPUT_DIR/implicit-ref-pruned-exp" "$OUTPUT_DIR/results-implicit-ref-pruned-exp.csv" milos-implicit-ref-pruned



######################################## unrefined graph ########################################
# no pruning
# if [ ! -d "$OUTPUT_DIR/implicit-unref-unpruned" ]; then
# 	EPSILONS=("1.0" "0.5" "0.25")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_ota_queries "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/implicit-unref-unpruned/$eps" "$QUERY_FILE" "$eps" "--pruning=none"
# 	done
# fi
# process_results "$OUTPUT_DIR/implicit-unref-unpruned" "$OUTPUT_DIR/results-implicit-unref-unpruned.csv" milos-implicit-unref-unpruned

# default pruning
# if [ ! -d "$OUTPUT_DIR/implicit-unref-pruned" ]; then
# 	EPSILONS=("1.0" "0.5" "0.25")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_ota_queries "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/implicit-unref-pruned/$eps" "$QUERY_FILE" "$eps" "--pruning=prune"
# 	done
# fi
# process_results "$OUTPUT_DIR/implicit-unref-pruned" "$OUTPUT_DIR/results-implicit-unref-pruned.csv" milos-implicit-unref-pruned
# # 
# # pruned by minimal bending angle
# if [ ! -d "$OUTPUT_DIR/implicit-unref-pruned-min-angle" ]; then
# 	EPSILONS=("1.0" "0.5" "0.25")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_ota_queries "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/implicit-unref--pruned-min-angle/$eps" "$QUERY_FILE" "$eps" "--pruning=prune-min-angle"
# 	done
# fi
# process_results "$OUTPUT_DIR/implicit-unref-pruned-min-angle" "$OUTPUT_DIR/results-implicit-unref-pruned-min-angle.csv" milos-implicit-unref-pruned-min-angle

######################################### explicit graph ########################################
# exact solutions
if [ ! -d "$OUTPUT_DIR/explicit-ref/1.0" ]; then
	compute_ota_queries "$EXPLICIT_REF_10_GRAPH" "$OUTPUT_DIR/explicit-ref/1.0" "$QUERY_FILE" 1.0 ""
	process_results "$OUTPUT_DIR/explicit-ref/1.0" "$OUTPUT_DIR/results-explicit-ref-10.csv" milos-explicit-ref-10
fi
if [ ! -d "$OUTPUT_DIR/explicit-ref/0.5" ]; then
	compute_ota_queries "$EXPLICIT_REF_05_GRAPH" "$OUTPUT_DIR/explicit-ref/0.5" "$QUERY_FILE" 0.5 ""
	process_results "$OUTPUT_DIR/explicit-ref/0.5" "$OUTPUT_DIR/results-explicit-ref-05.csv" milos-explicit-ref-05
fi
if [ ! -d "$OUTPUT_DIR/explicit-ref/0.25" ]; then
	compute_ota_queries "$EXPLICIT_REF_025_GRAPH" "$OUTPUT_DIR/explicit-ref/0.25" "$QUERY_FILE" 0.25 ""
	process_results "$OUTPUT_DIR/explicit-ref/0.25" "$OUTPUT_DIR/results-explicit-ref-025.csv" milos-explicit-ref-025
fi



######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
