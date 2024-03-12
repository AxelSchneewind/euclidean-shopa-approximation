#!/bin/bash

EXE=/path/to/route
EXPLICIT_GRAPH=/path/to/pruned_05.fmi
TRIANGLE_GRAPH=/path/to/pruned.graph

EXPLICIT_OUT=/path/to/results_explicit
IMPLICIT_OUT=/path/to/results


compute_explicit() {
$EXE --graph-file $EXPLICIT_GRAPH --output-directory $EXPLICIT_OUT --epsilon 0.5 --query 123000,100
}

compute_implicit() {
$EXE --graph-file $TRIANGLE_GRAPH --output-directory $IMPLICIT_OUT --epsilon 0.5 --query 123000,100
}

compute_explicit
compute_implicit
