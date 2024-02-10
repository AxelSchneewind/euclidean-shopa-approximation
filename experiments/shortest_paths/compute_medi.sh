#!/bin/bash

source utils.sh

# number of queries
NUM_QUERIES=100

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

#
MEDI_VISIBILITY_GRAPH=/opt/routing/graphs/medi/medi-ref-visibility.fmi
MEDI_TRIANGULATION_GRAPH=/opt/routing/graphs/medi/medi-ref.graph
compute "$MEDI_TRIANGULATION_GRAPH" "$MEDI_VISIBILITY_GRAPH" $NUM_QUERIES
process_results "$MEDI_TRIANGULATION_GRAPH"
