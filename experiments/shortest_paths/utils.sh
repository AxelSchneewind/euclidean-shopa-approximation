#!/bin/bash

# if not installed, add paths to executable here
ROUTER=route
FIND_NODES=find_nodes

compute_single() {
    if [[ -z "$4" ]]; then
	    echo "usage: compute_single path/to/graph_file.graph path/to/results/ path/to/queries.txt epsilon"
	    exit
    fi

    local GRAPH_FILE=$1
    local OUTPUT_DIR=$2
    local QUERY_FILE=$3
    local EPSILON=$4
    
    mkdir -p "$OUTPUT_DIR"
    
    # read queries
    local QUERIES="$(cat ${QUERY_FILE})"
    
    # run computations
    echo "$ROUTER --epsilon ${EPSILON} -p wgs84 -a $ASTAR -t$TREE_SIZE -l --graph-file ${GRAPH_FILE} --output-directory ${OUTPUT_DIR} --query ${QUERIES}"
    $ROUTER --epsilon "${EPSILON}" -p wgs84 -a "$ASTAR" -t$TREE_SIZE -l --graph-file "${GRAPH_FILE}" --output-directory "${OUTPUT_DIR}" --query "${QUERIES}" > "${OUTPUT_DIR}/out.log"
}

make_queries(){
    if [[ -z "$2" ]]; then
	    echo "usage: make_queries path/to/graph_file.graph path/to/queries.txt count"
	    exit
    fi

    local GRAPH_FILE=$1
    local QUERY_FILE=$2
    local NUM_QUERIES=${3:-100}

    touch "$QUERY_FILE"
    
    # select given number of nodes randomly from boundary nodes in triangulation
    local NUM_NODES=$((2 * NUM_QUERIES))
    echo "selecting $NUM_NODES nodes from graph $GRAPH_NAME..."
    local NODES=$($FIND_NODES -g "$GRAPH_FILE" -b -r $NUM_NODES)
    local NODES=$(echo "$NODES" | sed -z -E "s/\n/,/g")
    local NODES=${NODES::-1}
    echo "$NODES" > "$QUERY_FILE"
    echo "${NODES[@]}"
}

#
compute() {
    local GRAPH_FILE=$1
    local GRAPH_FILE_VISIBILITY=$2
    
    # twice the number of queries to run per graph
    local NUM_QUERIES=$3
    
    #
    local GRAPH_NAME=$(basename "$GRAPH_FILE")
    local GRAPH_NAME=${GRAPH_NAME%.graph}
    
    # make empty output directories
    local OUTPUT_DIR="results/${GRAPH_NAME}/approximate/"
    local OUTPUT_DIR_VIS="results/${GRAPH_NAME}/visibility/"
    local OUTPUT_DIR_RAW="results/${GRAPH_NAME}/raw/"
    mkdir -p "$OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR_VIS"
    mkdir -p "$OUTPUT_DIR_RAW"
    rm -rf "${OUTPUT_DIR:?}"/*
    rm -rf "${OUTPUT_DIR_VIS:?}"/*
    rm -rf "${OUTPUT_DIR_RAW:?}"/*
    
    # select given number of nodes randomly from boundary nodes in triangulation
    local NUM_NODES=$((2 * NUM_QUERIES))
    echo "selecting $NUM_NODES nodes from graph $GRAPH_NAME..."
    local NODES=$($FIND_NODES -g "$GRAPH_FILE" -b -r $NUM_NODES)
    local NODES=$(echo "$NODES" | sed -z -E "s/\n/,/g")
    local NODES=${NODES::-1}
    echo "$NODES" > "results/${GRAPH_NAME}/nodes.txt"
    echo "${NODES[@]}"
    
    # make queries
    local QUERY=""
    for i in "${NODES[@]}" ; do
      local QUERY+="$i,"
    done
    local QUERY=${QUERY::-1}
    
    # run shortest path computations
    echo " ################# computing approximate values without subdividing triangulation:  ################# "
    echo "$ROUTER --graph-file $GRAPH_FILE --output-directory $OUTPUT_DIR_RAW --epsilon inf --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
    $ROUTER --epsilon inf -p wgs84 -a "$ASTAR" -t$TREE_SIZE -l --graph-file "$GRAPH_FILE" --output-directory "$OUTPUT_DIR_RAW" --query "$QUERY"
    
    local EPSILONS=("1.0" "0.5" "0.2" "0.1" "0.05")
    # "0.02" "0.01")
    for eps in "${EPSILONS[@]}"; do
    echo " ########################## computing approximate values for epsilon=$eps: ########################## "
    echo "$ROUTER --graph-file $GRAPH_FILE --output-directory $OUTPUT_DIR --epsilon $eps --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
    $ROUTER --epsilon "$eps"  -p wgs84 -a "$ASTAR" -t$TREE_SIZE -l --graph-file "$GRAPH_FILE" --output-directory "$OUTPUT_DIR" --query "$QUERY"
    done
    
    echo " ##################################### computing exact values:  ##################################### "
    echo "$ROUTER --graph-file $GRAPH_FILE_VISIBILITY --output-directory $OUTPUT_DIR_VIS -e 0.0 --query $QUERY -p wgs84 -a $ASTAR -t$TREE_SIZE"
    $ROUTER -e 0.0 -p wgs84 -a "$ASTAR" -t$TREE_SIZE -l --graph-file "$GRAPH_FILE_VISIBILITY" --output-directory "$OUTPUT_DIR_VIS" --query "$QUERY"
}

process_results() {
    if [[ -z "$2" ]]; then
	    echo "usage: process_results path/to/results path/to/results.csv"
	    exit
    fi

    local OUTPUT_DIR="$1"
    local CSV_RESULTS="$2"
    
    local GRAPH_NAME=$(basename "$GRAPH_FILE")
    local GRAPH_NAME=${GRAPH_NAME%.graph}
    
    cat "$OUTPUT_DIR"/*/info.csv > "$CSV_RESULTS"

    # remove headers
    sed -e '1p;/,NODE.*/d' -i "$CSV_RESULTS"
    # remove ms unit for timings
    sed -e 's/ms//g' -i "$CSV_RESULTS"

    # add column with benchmark name
    sed -e '1,1s/$/,benchmark/' -i "$CSV_RESULTS"
    sed -e '2,$s/,$/,,OUTPUT_DIR/' -i "$CSV_RESULTS"
    local BENCH="${OUTPUT_DIR//\//\\\/}"
    local BENCH="${BENCH/results//}"
    sed -e "s/OUTPUT_DIR/$BENCH/" -i "$CSV_RESULTS"

    # echo "$(csvsort -c EPSILON,FROM,TO "$CSV_RESULTS")" > "$CSV_RESULTS"
}


process_all_results() {
    if [[ -z "$2" ]]; then
	    echo "usage: process_results path/to/results path/to/results.csv"
	    exit
    fi

    local OUTPUT_DIR="$1"
    local CSV_RESULTS="$2"

    cat "$OUTPUT_DIR"/results-*.csv > "$CSV_RESULTS"
    sed -e '1p;/,NODE.*/d' -i "$CSV_RESULTS"
}
