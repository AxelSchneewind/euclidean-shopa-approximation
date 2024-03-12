#!/bin/bash
#

GRAPH=/opt/routing/graphs/visibility_aegaeis/aegaeis-ref.graph
GRAPH_PATA=/opt/routing/graphs/pata/pata-ref.graph
GRAPH_MEDI=/opt/routing/graphs/medi/medi-ref.graph


subdivision_stats graph $GRAPH 8.0 --header | tee -a graph_sizes.csv
compute () {
for epsilon in 4.0 2.0 1.0 '1/2' '1/4' '1/8' '1/16' '1/32' '1/64' '1/128' '1/256'
do
  echo computing for ε = $epsilon ...
  # compute graph sizes
  subdivision_stats graph $1 $epsilon | tee -a graph_sizes.csv
done
}


compute $GRAPH
compute $GRAPH_PATA
compute $GRAPH_MEDI
