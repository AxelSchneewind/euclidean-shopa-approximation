#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=10

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
	make_ota_queries "$TRIANGULATION_REF_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
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
		local STORAGE_TXT="explicit"
		local BENCHMARK_NAME="milos-$STORAGE_TXT"
		local DIRECTORY_NAME="$OUTPUT_DIR/$BENCHMARK_NAME"
		local ARGUMENTS=""

		if [ ! -d "$DIRECTORY_NAME/1.0" ]; then
			compute_ota_queries "$EXPLICIT_REF_10_GRAPH" "$DIRECTORY_NAME/1.0" "$QUERY_FILE" 1.0 "$ARGUMENTS"
			process_results "$DIRECTORY_NAME/1.0" "$OUTPUT_DIR/results-$BENCHMARK_NAME-10.csv" "$BENCHMARK_NAME-10"
		fi
		if [ ! -d "$DIRECTORY_NAME/0.5" ]; then
			compute_ota_queries "$EXPLICIT_REF_05_GRAPH" "$DIRECTORY_NAME/0.5" "$QUERY_FILE" 0.5 "$ARGUMENTS"
			process_results "$DIRECTORY_NAME/0.5" "$OUTPUT_DIR/results-$BENCHMARK_NAME-05.csv" "$BENCHMARK_NAME-05"
		fi
		if [ ! -d "$DIRECTORY_NAME/0.25" ]; then
			compute_ota_queries "$EXPLICIT_REF_025_GRAPH" "$DIRECTORY_NAME/0.25" "$QUERY_FILE" 0.25 "$ARGUMENTS"
			process_results "$DIRECTORY_NAME/0.25" "$OUTPUT_DIR/results-$BENCHMARK_NAME-025.csv" "$BENCHMARK_NAME-025"
		fi

	else
		local STORAGE_TXT="implicit"
		local BENCHMARK_NAME="milos-$STORAGE_TXT-$PRUNING-$NEIGHBOR_FINDING"
		local DIRECTORY_NAME="$OUTPUT_DIR/$BENCHMARK_NAME"
		local ARGUMENTS="--pruning=$PRUNING --neighbor-finding=$NEIGHBOR_FINDING"

		# 
		if [[ "${STORAGE}" == "semi-explicit" ]]; then
			local STORAGE_TXT="semi-explicit"
			local BENCHMARK_NAME="milos-$STORAGE_TXT-$PRUNING-$NEIGHBOR_FINDING"
			local DIRECTORY_NAME="$OUTPUT_DIR/$BENCHMARK_NAME"
			local ARGUMENTS="--pruning $PRUNING --neighbor-finding $NEIGHBOR_FINDING --coords-explicit"
		fi

		EPSILONS=("1.0" "0.5" "0.25")
		for eps in "${EPSILONS[@]}"; do
			if [ ! -d "$DIRECTORY_NAME/$eps" ]; then
		    		compute_ota_queries "$GRAPH_FILE" "${DIRECTORY_NAME}/$eps" "$QUERIES" "$eps" "${ARGUMENTS}"
			fi
		done
		process_results "$DIRECTORY_NAME" "$OUTPUT_DIR/results-$BENCHMARK_NAME.csv" $BENCHMARK_NAME
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

######################################### explicit graph ########################################
# explicit solutions
compute_bench "$EXPLICIT_REF_10_GRAPH" "" "" "$QUERY_FILE"  "explicit"
compute_bench "$EXPLICIT_REF_05_GRAPH" "" "" "$QUERY_FILE"  "explicit"
compute_bench "$EXPLICIT_REF_025_GRAPH" "" "" "$QUERY_FILE"  "explicit"

######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
