#!/bin/bash

# if not installed, add paths to executable here
ROUTER=route
FIND_NODES=find_nodes

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
echo " ################# computing approximate values without subdividing triangulation:  ################# "
echo "$ROUTER --graph-file $GRAPH_FILE --output-directory $OUTPUT_DIR_RAW --epsilon inf --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
$ROUTER --graph-file "$GRAPH_FILE" --output-directory "$OUTPUT_DIR_RAW" --epsilon inf --query "$QUERY" -p wgs84 -a "$ASTAR" -t$TREE_SIZE

EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05" "0.02" "0.01")
for eps in "${EPSILONS[@]}"; do
echo " ########################## computing approximate values for epsilon=$eps: ########################## "
echo "$ROUTER --graph-file $GRAPH_FILE --output-directory $OUTPUT_DIR --epsilon $eps --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
$ROUTER --graph-file "$GRAPH_FILE" --output-directory "$OUTPUT_DIR" --epsilon "$eps" --query "$QUERY" -p wgs84 -a "$ASTAR" -t$TREE_SIZE
done

echo " ##################################### computing exact values:  ##################################### "
echo "$ROUTER --graph-file $GRAPH_FILE_VISIBILITY --output-directory $OUTPUT_DIR_VIS -e 0.0 --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
$ROUTER --graph-file "$GRAPH_FILE_VISIBILITY" --output-directory "$OUTPUT_DIR_VIS" -e 0.0 --query "$QUERY" -p wgs84 -a "$ASTAR" -t$TREE_SIZE

}

process_results() {
GRAPH_FILE=$1
CSV_RESULTS="results/$GRAPH_NAME/results.csv"

GRAPH_NAME=$(basename "$GRAPH_FILE")
GRAPH_NAME=${GRAPH_NAME%.graph}

cat "results/$GRAPH_NAME"/*/*/"info.csv" > "$CSV_RESULTS"
sed -e '1p;/,NODE.*/d' -i "$CSV_RESULTS"
sed -e 's/ms//g' -i "$CSV_RESULTS"
sed -e 's/ms//g' -i "$CSV_RESULTS"
# echo "$(csvsort -c EPSILON,FROM,TO "$CSV_RESULTS")" > "$CSV_RESULTS"
}
