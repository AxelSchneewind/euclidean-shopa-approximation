#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=1

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

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
	make_queries "$TRIANGULATION_REF_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi

# 
compute_bench() {
	local GRAPH_FILE="$1"
	local PRUNING="$2"
	local NEIGHBOR_FINDING="$3"
	local QUERIES="$4"
	local STORAGE="$5"

	# 
	if [[ "${GRAPH_FILE: -4}" == ".fmi" ]]; then
		local STORAGE="explicit"
		local BENCHMARK_NAME="milos-$STORAGE"
		local DIRECTORY_NAME="$OUTPUT_DIR/$BENCHMARK_NAME"
		local ARGUMENTS=""

		if [ ! -d "$DIRECTORY_NAME/1.0" ]; then
			compute_ota_queries "$EXPLICIT_REF_10_GRAPH" "$DIRECTORY_NAME/1.0" "$QUERY_FILE" 1.0 "$ARGUMENTS"
			process_results "$DIRECTORY_NAME" "$OUTPUT_DIR/$BENCHMARK_NAME.csv" "$BENCHMARK_NAME"
		fi
		if [ ! -d "$DIRECTORY_NAME/0.5" ]; then
			compute_ota_queries "$EXPLICIT_REF_05_GRAPH" "$DIRECTORY_NAME/0.5" "$QUERY_FILE" 0.5 "$ARGUMENTS"
			process_results "$DIRECTORY_NAME" "$OUTPUT_DIR/$BENCHMARK_NAME.csv" "$BENCHMARK_NAME"
		fi
		if [ ! -d "$DIRECTORY_NAME/0.25" ]; then
			compute_ota_queries "$EXPLICIT_REF_025_GRAPH" "$DIRECTORY_NAME/0.25" "$QUERY_FILE" 0.25 "$ARGUMENTS"
			process_results "$DIRECTORY_NAME" "$OUTPUT_DIR/$BENCHMARK_NAME.csv" "$BENCHMARK_NAME"
		fi

	else
		local STORAGE="implicit"
		local BENCHMARK_NAME="$STORAGE-$PRUNING-$NEIGHBOR_FINDING"
		local DIRECTORY_NAME="$OUTPUT_DIR/$BENCHMARK_NAME"
		local ARGUMENTS="--pruning=$PRUNING --neighbor-finding=$NEIGHBOR_FINDING"

		# 
		if [[ "${STORAGE}" == "semi-explicit" ]]; then
			local STORAGE="semi-explicit"
			local BENCHMARK_NAME="milos-$STORAGE-$PRUNING-$NEIGHBOR_FINDING"
			local DIRECTORY_NAME="$OUTPUT_DIR/$BENCHMARK_NAME"
			local ARGUMENTS="--pruning=$PRUNING --neighbor-finding=$NEIGHBOR_FINDING --coords-explicit"
		fi

		EPSILONS=("1.0" "0.5" "0.25")
		for eps in "${EPSILONS[@]}"; do
			if [ ! -d "$DIRECTORY_NAME/$eps" ]; then
		    		compute_shopa_queries "$GRAPH_FILE" "$DIRECTORY_NAME/$eps" "$QUERIES" "$eps" "$ARGUMENTS"
			fi
		done
		process_results "$DIRECTORY_NAME" "$OUTPUT_DIR/$BENCHMARK_NAME.csv" $BENCHMARK_NAME
	fi
}


############################ refined graph using triangle (Shewchuk) ############################
#
# GRAPH_FILE="$1"
# PRUNING="$2":  ("none", "prune", "prune-min-angle")
# NEIGHBOR_FINDING="$3": "param", "trig", "binary", "linear"
# QUERIES="$4"
# STORAGE="$5"
compute_bench "$TRIANGULATION_REF_GRAPH" "none" "param" "$QUERY_FILE"  "implicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "none" "trig" "$QUERY_FILE"   "implicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "none" "binary" "$QUERY_FILE" "implicit"

compute_bench "$TRIANGULATION_REF_GRAPH" "none" "param" "$QUERY_FILE"  "semi-explicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "none" "trig" "$QUERY_FILE"   "semi-explicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "none" "binary" "$QUERY_FILE" "semi-explicit"

compute_bench "$TRIANGULATION_REF_GRAPH" "prune" "param" "$QUERY_FILE"  "implicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune" "trig" "$QUERY_FILE"   "implicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune" "binary" "$QUERY_FILE" "implicit"

compute_bench "$TRIANGULATION_REF_GRAPH" "prune" "param" "$QUERY_FILE"  "semi-explicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune" "trig" "$QUERY_FILE"   "semi-explicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune" "binary" "$QUERY_FILE" "semi-explicit"

compute_bench "$TRIANGULATION_REF_GRAPH" "prune-min-angle" "param" "$QUERY_FILE"  "implicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune-min-angle" "trig" "$QUERY_FILE"   "implicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune-min-angle" "binary" "$QUERY_FILE" "implicit"

compute_bench "$TRIANGULATION_REF_GRAPH" "prune-min-angle" "param" "$QUERY_FILE"  "semi-explicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune-min-angle" "trig" "$QUERY_FILE"   "semi-explicit"
compute_bench "$TRIANGULATION_REF_GRAPH" "prune-min-angle" "binary" "$QUERY_FILE" "semi-explicit"

######################################## unrefined graph ########################################
# no pruning
# if [ ! -d "$OUTPUT_DIR/implicit-unref-unpruned" ]; then
# 	EPSILONS=("1.0" "0.5" "0.25")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_shopa_queries "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/implicit-unref-unpruned/$eps" "$QUERY_FILE" "$eps" "--pruning=none"
# 	done
# fi
# process_results "$OUTPUT_DIR/implicit-unref-unpruned" "$OUTPUT_DIR/results-implicit-unref-unpruned.csv" milos-implicit-unref-unpruned

# default pruning
# if [ ! -d "$OUTPUT_DIR/implicit-unref-pruned" ]; then
# 	EPSILONS=("1.0" "0.5" "0.25")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_shopa_queries "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/implicit-unref-pruned/$eps" "$QUERY_FILE" "$eps" "--pruning=prune"
# 	done
# fi
# process_results "$OUTPUT_DIR/implicit-unref-pruned" "$OUTPUT_DIR/results-implicit-unref-pruned.csv" milos-implicit-unref-pruned
# # 
# # pruned by minimal bending angle
# if [ ! -d "$OUTPUT_DIR/implicit-unref-pruned-min-angle" ]; then
# 	EPSILONS=("1.0" "0.5" "0.25")
# 	for eps in "${EPSILONS[@]}"; do
# 	    compute_shopa_queries "$TRIANGULATION_UNREF_GRAPH" "$OUTPUT_DIR/implicit-unref-pruned-min-angle/$eps" "$QUERY_FILE" "$eps" "--pruning=prune-min-angle"
# 	done
# fi
# process_results "$OUTPUT_DIR/implicit-unref-pruned-min-angle" "$OUTPUT_DIR/results-implicit-unref-pruned-min-angle.csv" milos-implicit-unref-pruned-min-angle

######################################### explicit graph ########################################
# exact solutions
compute_bench "$EXPLICIT_REF_10_GRAPH" "" "" "$QUERY_FILE"  "explicit"
compute_bench "$EXPLICIT_REF_05_GRAPH" "" "" "$QUERY_FILE"  "explicit"
compute_bench "$EXPLICIT_REF_025_GRAPH" "" "" "$QUERY_FILE"  "explicit"

######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
