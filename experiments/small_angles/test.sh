make_graph() {
mkdir -p $1_$2
echo "3
1
0.0 0.0
1.0 0.0
0.0 $2
0 1 2" > $1_$2/instance_$1_$2.graph
make_gl $1_$2/instance_$1_$2.graph $1_$2/instance_$1_$2.gl 1 1
subdivision_stats graph $1_$2/instance_$1_$2.graph 1/2 -h > $1_$2/stats.csv
}


rm results.csv

VAL=1.0
for i in {0..32}; do
	echo "$i, $VAL"
	make_graph triangle $VAL
	VAL=$(bc -l <<< "$VAL / 2")
done

csvstack */stats.csv > results.csv
sed -e 's/graph,/graph,sin,/' -e 's/triangle_/triangle,/' -i results.csv
