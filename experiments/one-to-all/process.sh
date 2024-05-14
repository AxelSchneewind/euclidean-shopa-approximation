#!/bin/bash

PYTHONPATH=/opt/routing/lib
export PYTHONPATH

PLOT_FILE_TYPE=.pgf

# create empty directory for summaries
# rm -rf summaries
mkdir summaries
mkdir summaries/aegaeis
mkdir summaries/pata
mkdir summaries/medi
# 
# # create empty directory for plots
# rm -rf plots
mkdir plots
mkdir plots/aegaeis
mkdir plots/pata
mkdir plots/medi
# 
# TODO output distributions of solution qualities per vertex

# triangle graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=triangle-pruned-min-angle -c=time --fliers -o plots/boxplots_time_triangle$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=triangle-pruned-min-angle -c='memory usage final' --fliers -o plots/boxplots_mem_triangle$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=triangle-pruned-min-angle -c='tree size' --fliers -o plots/boxplots_tree_triangle$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results.csv -t=triangle-pruned-min-angle --output-epsilon=summaries/triangle-epsilon.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/aegaeis/results.csv -t=triangle-pruned-min-angle --output-queries=summaries/triangle-queries.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/aegaeis/results.csv -t=triangle-pruned-min-angle --output-benchmark=summaries/triangle-benchmarks.csv --column 'time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'

# ref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=ref -c=time --fliers -o plots/boxplots_time_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=ref -c='memory usage final' --fliers -o plots/boxplots_mem_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=ref -c='tree size' --fliers -o plots/boxplots_tree_ref$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results.csv -t=ref --output-epsilon=summaries/ref-epsilon.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/aegaeis/results.csv -t=ref --output-queries=summaries/ref-queries.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/aegaeis/results.csv -t=ref --output-benchmark=summaries/ref-benchmarks.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'

# unref graph
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=unref -c=time --fliers -o plots/boxplots_time_unref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=unref -c='memory usage final' --fliers -o plots/boxplots_mem_unref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/aegaeis/results.csv -t=unref -c='tree size' --fliers -o plots/boxplots_tree_unref$PLOT_FILE_TYPE
python summarize.py -f results/aegaeis/results.csv -t=unref --output-epsilon=summaries/unref-epsilon.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/aegaeis/results.csv -t=unref --output-queries=summaries/unref-queries.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/aegaeis/results.csv -t=unref --output-benchmark=summaries/unref-benchmarks.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'


# ref graph: pata
# python plot_over_epsilon.py -f results/pata/results.csv -t=ref -c=time --fliers -o plots/pata/boxplots_time_ref$PLOT_FILE_TYPE
# python plot_over_epsilon.py -f results/pata/results.csv -t=ref -c='memory usage final' --fliers -o plots/pata/boxplots_mem_ref$PLOT_FILE_TYPE
# python plot_over_epsilon.py -f results/pata/results.csv -t=ref -c='tree size' --fliers -o plots/pata/boxplots_tree_ref$PLOT_FILE_TYPE
# python summarize.py -f results/pata/results.csv -t=ref --output-epsilon=summaries/pata/ref-epsilon.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
# python summarize.py -f results/pata/results.csv -t=ref --output-queries=summaries/pata/ref-queries.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
# python summarize.py -f results/pata/results.csv -t=ref --output-benchmark=summaries/pata/ref-benchmarks.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'

# ref graph: medi
python plot_over_epsilon.py -f results/medi/results.csv -t=ref -c=time --fliers -o plots/medi/boxplots_time_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/medi/results.csv -t=ref -c='memory usage final' --fliers -o plots/medi/boxplots_mem_ref$PLOT_FILE_TYPE
python plot_over_epsilon.py -f results/medi/results.csv -t=ref -c='tree size' --fliers -o plots/medi/boxplots_tree_ref$PLOT_FILE_TYPE
python summarize.py -f results/medi/results.csv -t=ref --output-epsilon=summaries/medi/ref-epsilon.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/medi/results.csv -t=ref --output-queries=summaries/medi/ref-queries.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/medi/results.csv -t=ref --output-benchmark=summaries/medi/ref-benchmarks.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'

# medi-vis
python summarize.py -f results/medi/results.csv -t=vis --output-epsilon=summaries/medi/vis-epsilon.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/medi/results.csv -t=vis --output-queries=summaries/medi/vis-queries.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'
python summarize.py -f results/medi/results.csv -t=vis --output-benchmark=summaries/medi/vis-benchmarks.csv --column='time,memory usage final,queue pull count,queue push count,neighbors base node neighbors count,neighbors boundary node neighbors count,neighbors steiner point neighbors count,tree size'






DISTANCE_SCRIPT=../explicit/distances.py

describe_distances() {
	local DIRECTORY=$1
	local REFERENCE_DIRECTORY=$2

	for q in "$DIRECTORY"*/ ; do
		QUERY=$(basename $q)
		echo "python $DISTANCE_SCRIPT" "$q"distances.csv "$REFERENCE_DIRECTORY""$QUERY"/distances.csv "$q"quality.csv
		python "$DISTANCE_SCRIPT" "$q"distances.csv "$REFERENCE_DIRECTORY""$QUERY"/distances.csv "$q"quality.csv
	done
}


# iterate over graphs
for g in results/*/ ; do
	for m in $g*/ ; do
		for e in $m*/ ; do
			DIR="$e"
			REFERENCE=$g/vis/0.0/
			describe_distances "$e" "$REFERENCE"
			csvstack $(find $e -name quality.csv) > $m/quality.csv
		done
	done
done

