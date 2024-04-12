#!/bin/bash

# if not installed, add paths to executable here
ROUTER=compute_shopa
ROUTER_OTA=compute_ota
FIND_NODES=find_nodes



compute_ota_queries() {
    if [[ -z "$4" ]]; then
	    echo "usage: compute_ota_queries path/to/graph_file.graph path/to/results/ path/to/queries.txt epsilon [other params]"
	    exit
    fi

    local GRAPH_FILE=$1
    local OUTPUT_DIR=$2
    local QUERY_FILE=$3
    local EPSILON=$4
    local PARAMS="$5"

    # check query file validity
    if [! -f "$QUERY_FILE"]; then
	    echo "invalid query file"
	    exit
    fi

    # check graph file validity
    if [! -f "$GRAPH_FILE"]; then
	    echo "invalid graph file"
	    exit
    fi

    mkdir -p "$OUTPUT_DIR"

    # read queries
    local QUERIES="$(cat ${QUERY_FILE})"

    # run computations
    echo "$ROUTER_OTA --epsilon ${EPSILON} ${PARAMS} -p wgs84 -a $ASTAR -t$TREE_SIZE -l --graph-file ${GRAPH_FILE} --output-directory ${OUTPUT_DIR} --query ${QUERIES}"
    $ROUTER_OTA --epsilon "${EPSILON}" ${PARAMS} -p wgs84 -a "$ASTAR" -t$TREE_SIZE -l --graph-file "${GRAPH_FILE}" --output-directory "${OUTPUT_DIR}" --query "${QUERIES}" > "${OUTPUT_DIR}/out.log" 2>&1
}

compute_shopa_queries() {
    if [[ -z "$4" ]]; then
	    echo "usage: compute_shopa_queries path/to/graph_file.graph path/to/results/ path/to/queries.txt epsilon [other params]"
	    exit
    fi

    local GRAPH_FILE=$1
    local OUTPUT_DIR=$2
    local QUERY_FILE=$3
    local EPSILON=$4
    local PARAMS="$5"

    # check query file validity
    if [! -f "$QUERY_FILE"]; then
	    echo "invalid query file"
	    exit
    fi

    # check graph file validity
    if [! -f "$GRAPH_FILE"]; then
	    echo "invalid graph file"
	    exit
    fi
    
    mkdir -p "$OUTPUT_DIR"
    
    # read queries
    local QUERIES="$(cat ${QUERY_FILE})"
    
    # run computations
    echo "$ROUTER --epsilon ${EPSILON} ${PARAMS} -p wgs84 -a $ASTAR -t$TREE_SIZE -l --graph-file ${GRAPH_FILE} --output-directory ${OUTPUT_DIR} --query ${QUERIES}"
    $ROUTER --epsilon "${EPSILON}" ${PARAMS} -p wgs84 -a "$ASTAR" -t$TREE_SIZE -l --graph-file "${GRAPH_FILE}" --output-directory "${OUTPUT_DIR}" --query "${QUERIES}" > "${OUTPUT_DIR}/out.log" 2>&1
}

make_queries(){
    if [[ -z "$2" ]]; then
	    echo "usage: make_queries path/to/graph_file.graph path/to/queries.txt count"
	    exit
    fi

    local GRAPH_FILE=$1
    local QUERY_FILE=$2
    local NUM_QUERIES=${3:-100}

    # check graph file validity
    if [! -f "$GRAPH_FILE"]; then
	    echo "invalid graph file"
	    exit
    fi

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



process_results() {
    if [[ -z "$3" ]]; then
	    echo "usage: process_results path/to/results path/to/results.csv benchmark_name"
	    exit
    fi

    local OUTPUT_DIR="$1"
    local CSV_RESULTS="$2"
    
    local GRAPH_NAME=$(basename "$GRAPH_FILE")
    local GRAPH_NAME=${GRAPH_NAME%.graph}
    local GRAPH_NAME=${GRAPH_NAME%.fmi}

    local NAME="${3:$GRAPH_NAME}"
    
    # cat all result files into one
    cat $(find $OUTPUT_DIR/ -name info.csv) > "$CSV_RESULTS"

    # remove headers
    sed -e '1p;/,node.*/d' -i "$CSV_RESULTS"
    # remove ms unit for timings
    sed -e 's/ms//g' -i "$CSV_RESULTS"

    # add columns with graph and benchmark name
    sed -e '1,1s/$/,graph/' -i "$CSV_RESULTS"
    sed -e '2,$s/$/,GRAPH/' -i "$CSV_RESULTS"
    sed -e "s/GRAPH/$GRAPH_NAME/" -i "$CSV_RESULTS"

    sed -e '1,1s/$/,benchmark/' -i "$CSV_RESULTS"
    sed -e '2,$s/$/,NAME/' -i "$CSV_RESULTS"
    sed -e "s/NAME/$NAME/" -i "$CSV_RESULTS"

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
    # remove headers
    sed -e '1p;/,node.*/d' -i "$CSV_RESULTS"
}
