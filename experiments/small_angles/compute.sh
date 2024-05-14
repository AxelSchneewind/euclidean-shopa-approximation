#!/bin/bash

# requires bc to be installed for floating point operations

EPSILON=("1.0" "0.5" "0.25" "0.125")

make_graph() {
mkdir -p results/$1_$2
# create triangle graph
echo -e "3\n1\n0.0 0.0\n1.0 0.0\n0.0 $2\n0 1 2" > results/$1_$2/instance_$1_$2.graph
make_gl results/$1_$2/instance_$1_$2.graph results/$1_$2/instance_$1_$2.gl 1 1

for e in "${EPSILON[@]}"; do
	graph_stats --mode steiner_graph_size  -g results/$1_$2/instance_$1_$2.graph -e $e > results/$1_$2/stats_$e.csv
done
}


# clear previous results and ensure directory exists
rm -f results.csv
mkdir -p results
rm -rf results/*

# iterate over sine values and compute graph size for each
VAL=1.0
for i in {0..48}; do
	echo -n -e "\r$i, $VAL"
	make_graph triangle $VAL
	VAL=$(bc -l <<< "$VAL / 2") # halve sin(alpha) here
done
echo -e "\r                              \rdone"

# combine all files into one
csvstack results/*/stats_*.csv > results/results.csv

# split graph column into graph and sin
sed -e 's/\.graph,/,/' -i results/results.csv
sed -e 's/,\./,0\./' -i results/results.csv
sed -e 's/graph,/graph,sin,/' -e 's/instance_triangle_/triangle,/' -i results/results.csv
echo "$(csvsort -c epsilon,sin -r results/results.csv)" > results/results.csv
sed -e 's/True/1/' -i results/results.csv
