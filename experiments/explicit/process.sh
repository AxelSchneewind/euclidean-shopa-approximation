#!/bin/bash
#


DISTANCE_SCRIPT=distances.py


describe_distances() {
	local DIRECTORY=$1
	local REFERENCE_DIRECTORY=$2

	for q in "$DIRECTORY"*/ ; do
		QUERY=$(basename $q)
		echo "python $DISTANCE_SCRIPT" "$q"distances.csv "$REFERENCE_DIRECTORY""$QUERY"/distances.csv "$q"quality.csv
		python distances.py "$q"distances.csv "$REFERENCE_DIRECTORY""$QUERY"/distances.csv "$q"quality.csv
	done
}


# iterate over graphs
for g in results/*/ ; do
	for m in $g*/ ; do
		for e in $m*/ ; do
			DIR="$e"
			DIR="${DIR/explicit-ref/implicit-ref-pruned}"
			DIR="${DIR/explicit-unref/implicit-unref-pruned}"
			DIR="${DIR/-min-angle/}"
			DIR="${DIR/-pruned/-unpruned}"
			REFERENCE=$DIR
			describe_distances "$e" "$REFERENCE"
		done
	done
done

csvstack $(find results -name quality.csv) > results/quality.csv
