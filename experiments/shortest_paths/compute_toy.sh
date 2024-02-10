#!/bin/bash

source utils.sh

# number of queries
NUM_QUERIES=6

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here (on/off)
ASTAR=on

#
TOY_VISIBILITY_GRAPH=/opt/routing/graphs/toy/toy-vis.fmi
TOY_TRIANGULATION_GRAPH=/opt/routing/graphs/toy/toy.graph
compute "$TOY_TRIANGULATION_GRAPH" "$TOY_VISIBILITY_GRAPH" $NUM_QUERIES
process_results "$TOY_TRIANGULATION_GRAPH"

