#!/bin/bash

GRAPH_DIR=/opt/routing/graphs
GRAPH_REF=$GRAPH_DIR/aegaeis/aegaeis-ref.graph
GRAPH_TRIANGLE=$GRAPH_DIR/aegaeis/aegaeis-ref-new.graph
GRAPH_UNREF=$GRAPH_DIR/aegaeis/aegaeis-unref.graph

#
# GRAPH_PATA=$GRAPH_DIR/pata/pata-ref.graph
# GRAPH_MEDI=$GRAPH_DIR/medi/medi-ref.graph


compute () {
  local GRAPH=$1
  local DIR=$2

  # check graph file
  if [! -f "$GRAPH" ]; then
    echo "invalid graph file"
    exit
  fi

  # check dir argument
  if [ -z "$DIR" ]; then
    echo "invalid output directory"
    exit
  fi

  mkdir -p "$DIR"

  graph_stats -g $GRAPH -m inangle_distribution > $DIR/inangles.csv
  graph_stats -g $GRAPH -m points_per_edge > $DIR/points_per_edge.csv
  graph_stats -g $GRAPH -m node_radii > $DIR/radii.csv
}


compute $GRAPH_REF results/aegaeis/ref/
compute $GRAPH_TRIANGLE results/aegaeis/triangle/
compute $GRAPH_UNREF results/aegaeis/unref/
# compute $GRAPH_PATA results/pata/ref
# compute $GRAPH_MEDI results/medi/ref
