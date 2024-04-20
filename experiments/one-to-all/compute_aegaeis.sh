#!/bin/bash

source ../utils.sh

# number of queries
NUM_QUERIES=20

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# graph files
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
  make_ota_queries "$UNREF_TRIANGULATION_GRAPH" "$QUERY_FILE" "$NUM_QUERIES"
fi

########################################## refined graph #########################################
## pruned
if [ ! -d "$OUTPUT_DIR/ref" ]; then
  # refined graph without points
  compute_ota_queries "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref/inf/" "$QUERY_FILE" inf "--pruning=prune"

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$TRIANGULATION_GRAPH" "$OUTPUT_DIR/ref/$eps/" "$QUERY_FILE" "$eps" "--pruning=prune"
  done
fi
process_results "$OUTPUT_DIR/ref/" "$OUTPUT_DIR/results-ref.csv" aegaeis-ref

############################ refined graph using triangle (Shewchuk) ############################
## with pruning
if [ ! -d "$OUTPUT_DIR/triangle-pruned" ]; then
  # refined graph without points
  compute_ota_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle-pruned/inf/" "$QUERY_FILE" inf "--pruning=prune"

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25" "0.125" "0.0625" "0.03125" "0.015625")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle-pruned/$eps/" "$QUERY_FILE" "$eps" "--pruning=prune"
  done
fi
process_results "$OUTPUT_DIR/triangle-pruned/" "$OUTPUT_DIR/results-triangle-pruned.csv" aegaeis-triangle-pruned

## without pruning
if [ ! -d "$OUTPUT_DIR/triangle-unpruned" ]; then
  # refined graph without points
  compute_ota_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle-unpruned/inf/" "$QUERY_FILE" inf "--pruning=none"

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25" "0.125" "0.0625" "0.03125" "0.015625")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle-unpruned/$eps/" "$QUERY_FILE" "$eps" "--pruning=none"
  done
fi
process_results "$OUTPUT_DIR/triangle-unpruned/" "$OUTPUT_DIR/results-triangle-unpruned.csv" aegaeis-triangle-unpruned

## with pruning based on minimal bending angles
if [ ! -d "$OUTPUT_DIR/triangle-pruned-min-angle" ]; then
  # refined graph without points
  compute_ota_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle-pruned-min-angle/inf/" "$QUERY_FILE" inf "--pruning=prune-min-angle"

  # refined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25" "0.125" "0.0625" "0.03125" "0.015625")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$TRIANGLE_TRIANGULATION_GRAPH" "$OUTPUT_DIR/triangle-pruned-min-angle/$eps/" "$QUERY_FILE" "$eps" "--pruning=prune-min-angle"
  done
fi
process_results "$OUTPUT_DIR/triangle-pruned-min-angle/" "$OUTPUT_DIR/results-triangle-pruned-min-angle.csv" aegaeis-triangle-pruned-min-angle


######################################## unrefined graph ########################################
## pruned
if [ ! -d "$OUTPUT_DIR/unref-pruned" ]; then
  # unrefined graph raw
  compute_ota_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-pruned/inf/" "$QUERY_FILE" inf "--pruning=prune"

  # unrefined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-pruned/$eps/" "$QUERY_FILE" "$eps" "--pruning=prune"
  done
fi
process_results "$OUTPUT_DIR/unref-pruned/" "$OUTPUT_DIR/results-unref-pruned.csv" aegaeis-unref-pruned

## unpruned
if [ ! -d "$OUTPUT_DIR/unref-unpruned" ]; then
  # unrefined graph raw
  compute_ota_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-unpruned/inf/" "$QUERY_FILE" inf "--pruning=none"

  # unrefined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-unpruned/$eps/" "$QUERY_FILE" "$eps" "--pruning=none"
  done
fi
process_results "$OUTPUT_DIR/unref-unpruned/" "$OUTPUT_DIR/results-unref-unpruned.csv" aegaeis-unref-unpruned

## pruned by minimal bending angles
if [ ! -d "$OUTPUT_DIR/unref-prune-min-angle" ]; then
  # unrefined graph raw
  compute_ota_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-prune-min-angle/inf/" "$QUERY_FILE" inf "--pruning=prune-min-angle"

  # unrefined graph with steiner points
  EPSILONS=("1.0" "0.5" "0.25")
  for eps in "${EPSILONS[@]}"; do
    compute_ota_queries "$UNREF_TRIANGULATION_GRAPH" "$OUTPUT_DIR/unref-prune-min-angle/$eps/" "$QUERY_FILE" "$eps" "--pruning=prune-min-angle"
  done
fi
process_results "$OUTPUT_DIR/unref-prune-min-angle/" "$OUTPUT_DIR/results-unref-prune-min-angle.csv" aegaeis-unref-prune-min-angle



######################################## visibility graph #######################################
# exact solutions
if [ ! -d "$OUTPUT_DIR/vis" ]; then
  compute_ota_queries "$VISIBILITY_GRAPH" "$OUTPUT_DIR/vis/0.0/" "$QUERY_FILE" 0.0
fi
process_results "$OUTPUT_DIR/vis/" "$OUTPUT_DIR/results-exact.csv" aegaeis-exact

######################################### postprocessing ########################################
# aggregate results into one file
process_all_results "$OUTPUT_DIR" "$OUTPUT_DIR"/results.csv
