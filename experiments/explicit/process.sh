#!/bin/bash
#


DISTANCE_SCRIPT=distances.py


describe_distances() {
	local DIRECTORY=$1
	local REFERENCE_DIRECTORY=$2

	for q in "$DIRECTORY"*/ ; do
		QUERY=$(basename $q)
		# echo "python $DISTANCE_SCRIPT" "$q"distances.csv "$REFERENCE_DIRECTORY""$QUERY"/distances.csv "$q"quality.csv
		echo "$q"distances.csv "$REFERENCE_DIRECTORY""$QUERY"/distances.csv

		if [[ ! -f "$REFERENCE_DIRECTORY""$QUERY"/distances.csv ]]; then
			exit
		fi

		if [[ ! -f "$q"distances.csv ]]; then
			exit
		fi

		if [[ ! -f "$q"quality.csv ]]; then
			python distances.py "$q"distances.csv "$REFERENCE_DIRECTORY""$QUERY"/distances.csv "$q"quality.csv
		fi
	done
	echo "csvstack $(find "$DIRECTORY" -name quality.csv | tr '\n' ' ') > "$DIRECTORY"/quality.csv"
	csvstack $(find "$DIRECTORY" -name quality.csv | tr '\n' ' ') > "$DIRECTORY"/quality.csv
	python reduce.py "$DIRECTORY"/quality.csv > "$DIRECTORY"/mean_max.csv

	# add directory column
	sed -e 's/mean,max/mean,max,path,reference/' -i "$DIRECTORY"/mean_max.csv
	echo ",$DIRECTORY" >> "$DIRECTORY"/mean_max.csv
	echo ",$REFERENCE_DIRECTORY" >> "$DIRECTORY"/mean_max.csv
	sed -z -e 's/\n,/,/' -i "$DIRECTORY"/mean_max.csv
	sed -z -e 's/\n,/,/' -i "$DIRECTORY"/mean_max.csv

	if [[ "$(wc -l $DIRECTORY/mean_max.csv)" == "0" ]]; then
		exit
	fi
}


# iterate over graphs
for g in results/*/ ; do
	for m in $g*/ ; do
		for e in $m*/ ; do
			DIR="$e"
			DIR="${DIR/semi-explicit/implicit}"
			DIR="${DIR/explicit/implicit-none-binary}"
			DIR="${DIR/prune-min-angle/none}"
			DIR="${DIR/prune/none}"
			DIR="${DIR/binary/param}"
			DIR="${DIR/trig/param}"
			REFERENCE=$DIR
			describe_distances "$e" "$REFERENCE"
		done
	done
	csvstack $(find $g -name quality.csv) > $g/quality.csv
	csvstack $(find $g -name mean_max.csv) > $g/mean_max.csv
done

echo "success!"
