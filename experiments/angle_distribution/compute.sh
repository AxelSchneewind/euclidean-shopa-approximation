#!/bin/bash
#

GRAPH_REF=/opt/routing/graphs/aegaeis/aegaeis-ref.graph
GRAPH_TRIANGLE=/opt/routing/graphs/aegaeis/aegaeis-ref-new.graph
GRAPH_UNREF=/opt/routing/graphs/aegaeis/aegaeis-unref.graph

#
# GRAPH_PATA=/opt/routing/graphs/pata/pata-ref.graph
# GRAPH_MEDI=/opt/routing/graphs/medi/medi-ref.graph


compute () {
  local GRAPH=$1
  local FILE=$2
  graph_stats -g $GRAPH -m inangle_distribution > $FILE
}


compute $GRAPH_REF aegaeis-ref.csv
compute $GRAPH_TRIANGLE aegaeis-triangle.csv
compute $GRAPH_UNREF aegaeis-unref.csv
compute $GRAPH_PATA pata-ref.csv
compute $GRAPH_MEDI medi-ref.csv
