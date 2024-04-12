#!/bin/bash

source ../utils.sh

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
  if [[ ! -f "$GRAPH" ]]; then
    echo "invalid graph file"
    exit
  fi

  # check dir argument
  if [[ -z "$DIR" ]]; then
    echo "invalid output directory"
    exit
  fi

  mkdir -p "$DIR"

  graph_stats -g $GRAPH -m inangle_distribution > $DIR/inangles.csv
  graph_stats -g $GRAPH -m node_radii > $DIR/radii.csv

  graph_stats -g $GRAPH -m points_per_edge -e 1.0   -b 50 > $DIR/points_per_edge-10.csv
  graph_stats -g $GRAPH -m points_per_edge -e 0.5   -b 50 > $DIR/points_per_edge-05.csv
  graph_stats -g $GRAPH -m points_per_edge -e 0.25  -b 50 > $DIR/points_per_edge-025.csv
  graph_stats -g $GRAPH -m points_per_edge -e 0.125 -b 50 > $DIR/points_per_edge-0125.csv
}


compute $GRAPH_REF results/aegaeis/ref/
compute $GRAPH_TRIANGLE results/aegaeis/triangle/
compute $GRAPH_UNREF results/aegaeis/unref/
# compute $GRAPH_PATA results/pata/ref
# compute $GRAPH_MEDI results/medi/ref
