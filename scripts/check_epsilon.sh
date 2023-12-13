#!/bin/bash

ROUTER=/home/axel/Studium/Semester_7/routing/cmake-build-release/route

VISIBILITY_GRAPH=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis-visibility.fmi
TRIANGULATION_GRAPH=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/aegaeis10-ref.graph

VISIBILITY_RESULTS=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/results
TRIANGULATION_RESULTS=/home/axel/Studium/Semester_7/graphs/visibility_aegaeis/results

NODES=(100 20000 100000 200000 500000 56961)

# setup query to perform computation for each pair of nodes
QUERY=""
for i in "${NODES[@]}" ; do
  for j in "${NODES[@]}" ; do
    QUERY+="$i,$j,"
  done
done
QUERY=${QUERY::-1}

echo "##################### computing exact values: #####################"
echo "$ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --epsilon 0.0 --query $QUERY" -c
$ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --epsilon 0.0 --query $QUERY -c | tee --append out.log

echo "################## computing approximate values: ##################"
echo "$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.2 --query $QUERY" -c
$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.2 --query $QUERY -c | tee --append out.log
