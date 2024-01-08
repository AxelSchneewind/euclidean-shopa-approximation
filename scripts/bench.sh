#!/bin/bash

ROUTER=/opt/routing/bin/route
ROUTING_FLAGS=""

VISIBILITY_GRAPH=/opt/routing/graphs/visibility_aegaeis/aegaeis-visibility.fmi
TRIANGULATION_GRAPH=/opt/routing/graphs/visibility_aegaeis/aegaeis10-ref-projected.graph
TRIANGULATION_UNREF_GRAPH=/opt/routing/graphs/visibility_aegaeis/aegaeis10-unref.graph

VISIBILITY_RESULTS=./results_exact
TRIANGULATION_RESULTS=./results_steiner
TRIANGULATION_UNREF_RESULTS=./results_unref

# make result directories
mkdir -p $VISIBILITY_RESULTS
mkdir -p $TRIANGULATION_RESULTS
mkdir -p $TRIANGULATION_UNREF_RESULTS

# clear previous results
rm -r $VISIBILITY_RESULTS/*
rm -r $TRIANGULATION_RESULTS/*
rm -r $TRIANGULATION_UNREF_RESULTS/*

# set nodes for route computationns
NODES=(100 20000 100000 123000 200000 500000 56961)

# setup query to perform computation for each pair of nodes
QUERY=""
for i in "${NODES[@]}" ; do
  for j in "${NODES[@]}" ; do
    QUERY+="$i,$j,"
  done
done
QUERY=${QUERY::-1}

# echo "##################### computing exact values: #####################"
# echo "########################## exact values: ##########################" >> out.log
# echo "$ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --query $QUERY"
# $ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --query $QUERY $ROUTING_FLAGS | tee --append out.log


echo "################## computing approximate values: ##################"
echo "####################### approximate values: #######################" >> out.log
echo "$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.5 --query $QUERY"
$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.5 --query $QUERY -p wgs84 $ROUTING_FLAGS | tee --append out.log


# for displaying all paths
# render_graph $(echo ${"$(echo $TRIANGULATION_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $TRIANGULATION_UNREF_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $VISIBILITY_RESULTS/*/path.gl)"//\ /\ -gf\ })
