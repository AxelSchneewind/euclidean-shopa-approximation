#!/bin/bash

ROUTER=route
FIND_NODES=find_nodes

VISIBILITY_GRAPH=pata-ref-visibility.fmi
TRIANGULATION_GRAPH=pata-ref.graph

VISIBILITY_RESULTS=results-visibility/
TRIANGULATION_RESULTS=results-approximate/


NUM_NODES=50

# make query by selecting some random boundary nodes from the triangulation
QUERY=$($FIND_NODES -g $TRIANGULATION_GRAPH -b -r $NUM_NODES -s 1234)
QUERY=$(echo "$QUERY" | sed -z -E "s/\n/,/g")
QUERY=${QUERY::-1}
echo "$QUERY" > query.txt

# clear previous results
rm -r $VISIBILITY_RESULTS 2> /dev/null
rm -r $TRIANGULATION_RESULTS 2> /dev/null
rm out.log
rm results.csv

# make result directories
mkdir -p $VISIBILITY_RESULTS
mkdir -p $TRIANGULATION_RESULTS


echo "################### computing cost for nodes: ####################"
echo "$QUERY"

# echo "##################### computing exact values: #####################"
# echo "########################## exact values: ##########################" >> out.log
# echo "$ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --query $QUERY"
# $ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --query $QUERY -a | tee --append out.log

echo "################## computing approximate values: ##################"
echo "####################### approximate values: #######################" >> out.log
echo "$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.5 --query $QUERY"
$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.5 --query $QUERY -p wgs84 -a | tee --append out.log

# render_graph $(echo ${"$(echo $TRIANGULATION_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $TRIANGULATION_UNREF_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $VISIBILITY_RESULTS/*/path.gl)"//\ /\ -gf\ })
