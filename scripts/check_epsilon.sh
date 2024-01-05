#!/bin/bash

ROUTER=route

VISIBILITY_GRAPH=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis-visibility.fmi
TRIANGULATION_GRAPH=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis10-ref-projected.graph
TRIANGULATION_UNREF_GRAPH=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis10-unref.graph

VISIBILITY_RESULTS=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/results_exact
TRIANGULATION_RESULTS=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/results_steiner
TRIANGULATION_UNREF_RESULTS=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/results_unref


NODES=(100 20000 100000 123000 200000 500000 56961)
# TODO
NODES_UNREF=()

# setup query to perform computation for each pair of nodes
QUERY=""
for i in "${NODES[@]}" ; do
  for j in "${NODES[@]}" ; do
    QUERY+="$i,$j,"
  done
done
QUERY=${QUERY::-1}

# QUERY_UNREF=""
# for i in "${NODES_UNREF[@]}" ; do
#   for j in "${NODES_UNREF[@]}" ; do
#     QUERY+="$i,$j,"
#   done
# done
# QUERY_UNREF=${QUERY_UNREF::-1}

# echo "##################### computing exact values: #####################"
# echo "########################## exact values: ##########################" >> out.log
# echo "$ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --epsilon 0.0 --query $QUERY" -c
# $ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --epsilon 0.0 --query $QUERY -c | tee --append out.log


# echo "########## computing approximate values in unref graph:  ##########"
# echo "############### approximate values in unref graph:  ###############" >> out.log
# echo "$ROUTER --graph-file $TRIANGULATION_UNREF_GRAPH --output-directory $TRIANGULATION_UNREF_RESULTS --epsilon 0.2 --query $QUERY_UNREF" -c
# $ROUTER --graph-file $TRIANGULATION_UNREF_GRAPH --output-directory $TRIANGULATION_UNREF_RESULTS --epsilon 0.2 --query $QUERY_UNREF -c | tee --append out.log

echo "################## computing approximate values: ##################"
echo "####################### approximate values: #######################" >> out.log
echo "$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.2 --query $QUERY" -c
$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.2 --query $QUERY -p wgs84 | tee --append out.log
render_graph $(echo ${"$(echo ../../graphs/visibility_aegaeis/results_steiner/*/path.gl)"//\ /\ -gf\ })
