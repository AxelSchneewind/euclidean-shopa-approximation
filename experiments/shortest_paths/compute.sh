#!/bin/bash

# if not installed, add paths to executable here
ROUTER=route
FIND_NODES=find_nodes

# 
AEGS_VISIBILITY_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-ref-vis.fmi
AEGS_TRIANGULATION_GRAPH=/opt/routing/graphs/aegaeis/aegaeis-ref.graph
PATA_VISIBILITY_GRAPH=/opt/routing/graphs/pata/pata-ref-vis.fmi
PATA_TRIANGULATION_GRAPH=/opt/routing/graphs/pata/pata-ref.graph
MEDI_VISIBILITY_GRAPH=/opt/routing/graphs/medi/medi-ref-visibility.fmi
MEDI_TRIANGULATION_GRAPH=/opt/routing/graphs/medi/medi-ref.graph

# maximum tree size to write to files (0 to disable tree output)
TREE_SIZE=0

# toggle A* here
ASTAR=on

#
compute() {
GRAPH_FILE=$1
GRAPH_FILE_VISIBILITY=$2

# twice the number of queries to run per graph
NUM_QUERIES=$3

#
GRAPH_NAME=$(basename "$GRAPH_FILE")
GRAPH_NAME=${GRAPH_NAME%.graph}

# make empty output directories
OUTPUT_DIR="results/${GRAPH_NAME}/approximate/"
OUTPUT_DIR_VIS="results/${GRAPH_NAME}/visibility/"
OUTPUT_DIR_RAW="results/${GRAPH_NAME}/raw/"
mkdir -p "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR_VIS"
mkdir -p "$OUTPUT_DIR_RAW"
rm -rf "${OUTPUT_DIR:?}"/*
rm -rf "${OUTPUT_DIR_VIS:?}"/*
rm -rf "${OUTPUT_DIR_RAW:?}"/*

# select given number of nodes randomly from boundary nodes in triangulation
NUM_NODES=$((2 * NUM_QUERIES))
echo "selecting $NUM_NODES nodes from graph $GRAPH_NAME..."
NODES=$($FIND_NODES -g "$GRAPH_FILE" -b -r $NUM_NODES)
NODES=$(echo "$NODES" | sed -z -E "s/\n/,/g")
NODES=${NODES::-1}
echo "$NODES" > "results/${GRAPH_NAME}/nodes.txt"
echo "${NODES[@]}"

# make queries
QUERY=""
for i in "${NODES[@]}" ; do
  QUERY+="$i,"
done
QUERY=${QUERY::-1}

# run shortest path computations
echo " ##################################### computing exact values:  ##################################### "
echo "$ROUTER --graph-file $GRAPH_FILE_VISIBILITY --output-directory $OUTPUT_DIR_VIS -e 0.0 --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
$ROUTER --graph-file "$GRAPH_FILE_VISIBILITY" --output-directory "$OUTPUT_DIR_VIS" -e 0.0 --query "$QUERY" -p wgs84 -a "$ASTAR" -t$TREE_SIZE

EPSILONS=("1.0" "0.5" "0.2")
for eps in "${EPSILONS[@]}"; do
echo " ########################## computing approximate values for epsilon=$eps: ########################## "
echo "$ROUTER --graph-file $GRAPH_FILE --output-directory $OUTPUT_DIR --epsilon $eps --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
$ROUTER --graph-file "$GRAPH_FILE" --output-directory "$OUTPUT_DIR" --epsilon "$eps" --query "$QUERY" -p wgs84 -a "$ASTAR" -t$TREE_SIZE
done

echo " ################# computing approximate values without subdividing triangulation:  ################# "
echo "$ROUTER --graph-file $GRAPH_FILE --output-directory $OUTPUT_DIR_RAW --epsilon inf --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
$ROUTER --graph-file "$GRAPH_FILE" --output-directory "$OUTPUT_DIR_RAW" --epsilon inf --query "$QUERY" -p wgs84 -a "$ASTAR" -t$TREE_SIZE

}

compute "$AEGS_TRIANGULATION_GRAPH" "$AEGS_VISIBILITY_GRAPH" 20
compute "$PATA_TRIANGULATION_GRAPH" "$PATA_VISIBILITY_GRAPH" 20
compute "$MEDI_TRIANGULATION_GRAPH" "$MEDI_VISIBILITY_GRAPH" 20


# to display all paths/beelines at once
# render_graph $(echo ${"$(echo $TRIANGULATION_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $TRIANGULATION_RAW_RESULTS/*/path.gl)"//\ /\ -gf\ })
# render_graph $(echo ${"$(echo $VISIBILITY_RESULTS/*/path.gl)"//\ /\ -gf\ })
#
