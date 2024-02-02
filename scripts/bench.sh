#!/bin/bash

# if not installed, add paths to executable here
ROUTER=route
FIND_NODES=find_nodes

# 
VISIBILITY_GRAPH=pata-ref-visibility.fmi
TRIANGULATION_GRAPH=pata-ref.graph

VISIBILITY_RESULTS=results-visibility/
TRIANGULATION_RESULTS=results-approximate/
TRIANGULATION_RAW_RESULTS=results-raw/

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# twice the number of queries to run
NUM_NODES=20

# select given number of nodes randomly from boundary nodes in triangulation
echo "selecting $NUM_NODES from graph..."
NODES=$($FIND_NODES -g $TRIANGULATION_GRAPH -b -r $NUM_NODES)
NODES=$(echo "$NODES" | sed -z -E "s/\n/,/g")
NODES=${NODES::-1}
echo "$NODES" > nodes.txt
echo "${NODES[@]}"

# make queries
QUERY=""
for i in "${NODES[@]}" ; do
  QUERY+="$i,"
done
QUERY=${QUERY::-1}


echo "######################## computing exact values: #########################"
echo "############################ exact values: ###############################" >> out.log
echo "$ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --query $QUERY -t$TREE_SIZE
$ROUTER --graph-file $VISIBILITY_GRAPH --output-directory $VISIBILITY_RESULTS --query $QUERY -e 0.0 -t$TREE_SIZE

echo "##################### computing approximate values: ######################"
echo "########################## approximate values: ###########################" >> out.log
echo "$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.5 --query $QUERY -p wgs84 -a on -t$TREE_SIZE
$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RESULTS --epsilon 0.5 --query $QUERY -p wgs84 -a on -t$TREE_SIZE

echo "##### computing approximate values without subdividing triangulation: #####"
echo "######################### raw approximate values: #########################" >> out.log
echo "$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RAW_RESULTS --epsilon inf --query $QUERY -p wgs84 -a on -t$TREE_SIZE
$ROUTER --graph-file $TRIANGULATION_GRAPH --output-directory $TRIANGULATION_RAW_RESULTS --epsilon inf --query $QUERY -p wgs84 -a on -t$TREE_SIZE



# to display all paths/beelines at once
# render_graph $(echo ${"$(echo $TRIANGULATION_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $TRIANGULATION_RAW_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $VISIBILITY_RESULTS/*/path.gl)"//\ /\ -gf\ })
