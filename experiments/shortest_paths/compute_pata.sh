#!/bin/bash

source utils.sh

# number of queries
NUM_QUERIES=100

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

#
PATA_VISIBILITY_GRAPH=/opt/routing/graphs/pata/pata-ref-vis.fmi
PATA_TRIANGULATION_GRAPH=/opt/routing/graphs/pata/pata-ref.graph
compute "$PATA_TRIANGULATION_GRAPH" "$PATA_VISIBILITY_GRAPH" $NUM_QUERIES
process_results "$PATA_TRIANGULATION_GRAPH"
