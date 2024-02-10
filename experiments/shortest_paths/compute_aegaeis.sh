#!/bin/bash

source utils.sh

# number of queries
NUM_QUERIES=100

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

# 
AEGS_VISIBILITY_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-ref-vis.fmi
AEGS_TRIANGULATION_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-ref.graph
compute "$AEGS_TRIANGULATION_GRAPH" "$AEGS_VISIBILITY_GRAPH" $NUM_QUERIES
process_results "$AEGS_TRIANGULATION_GRAPH"
