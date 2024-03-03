#!/bin/bash


ROUTER=route
MAKE_GL=make_gl
PRUNE_GRAPH=prune_graph
PROJECT=project_graph

GRAPH_DIR=/opt/routing/graphs/
INPUT_GRAPH=$GRAPH_DIR/visibility_aegaeis/aegaeis10-ref-projected.graph


echo 'making gl'
OUTPUT_GRAPH
OUTPUT_GRAPH_GL=aegaeis10-skala.gl
$PRUNE_GRAPH $INPUT_GRAPH $OUTPUT_GRAPH_GL 2.0 26.5 26.8 37.3 37.4

# test if routing works with high epsilon
echo 'Testing router with ε = 2.0'
$ROUTER --graph-file $INPUT_GRAPH --output-directory $GRAPH_DIR/results-ref/ --epsilon 2.0 --q 123000,100 --projection wgs84

# test if routing works with high epsilon
echo 'Testing router with ε = 0.2'
$ROUTER --graph-file $INPUT_GRAPH --output-directory $GRAPH_DIR/results-ref/ --epsilon 0.2 --q 123000,100 --projection wgs84


