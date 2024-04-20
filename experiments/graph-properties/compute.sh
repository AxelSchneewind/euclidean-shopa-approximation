#!/bin/bash

source ../utils.sh

GRAPH_REF=$GRAPH_DIR/aegaeis/aegaeis-ref.graph
GRAPH_TRIANGLE=$GRAPH_DIR/aegaeis/aegaeis-ref-new.graph
GRAPH_UNREF=$GRAPH_DIR/aegaeis/aegaeis-unref.graph

#
MILOS_REF=../explicit/milos-ref.graph
MILOS_UNREF=../explicit/milos.graph

#
GRAPH_PATA=$GRAPH_DIR/pata/pata-ref.graph
GRAPH_MEDI=$GRAPH_DIR/medi/medi-ref.graph
GRAPH_COASTLINES=$GRAPH_DIR/coastlines/coastlines.graph


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
  graph_stats -g $GRAPH -m bounding_box > $DIR/bounding_box.txt
}


compute $GRAPH_REF results/aegaeis/ref/
compute $GRAPH_TRIANGLE results/aegaeis/triangle/
compute $GRAPH_UNREF results/aegaeis/unref/

compute $MILOS_REF results/milos/ref/
compute $MILOS_UNREF results/milos/unref/

compute $GRAPH_PATA results/pata/ref
compute $GRAPH_MEDI results/medi/ref
compute $GRAPH_COASTLINES results/coastlines/ref
