#!/bin/bash


TRIANGLE_EXE=/opt/routing/graphs/aegaeis/triangle/triangle

FILE_IN_UNREF=/opt/routing/graphs/aegaeis/aegaeis-unref.graph
FILE_IN_REF=/opt/routing/graphs/aegaeis/aegaeis-ref-new.graph

FILE_IN_UNREF_PROJ=aegaeis-unref-latlon.graph
FILE_IN_REF_PROJ=aegaeis-ref-latlon.graph

FILE_UNREF=pruned.graph
FILE_REF=pruned-ref.graph

FILE_UNREF_GL=pruned.gl
FILE_REF_GL=pruned-ref.gl

FILE_REF_EXPLICIT_10=pruned-ref-10.fmi
FILE_REF_EXPLICIT_05=pruned-ref-05.fmi
FILE_REF_EXPLICIT_025=pruned-ref-025.fmi
FILE_UNREF_EXPLICIT=pruned-unref.fmi

FILE_REF_EXPLICIT_GL_10=pruned-ref-explicit-10.gl
FILE_REF_EXPLICIT_GL_05=pruned-ref-explicit-05.gl
FILE_REF_EXPLICIT_GL_025=pruned-ref-explicit-025.gl


# bounding box for milos island
MINY=36.50861358642578125000
MAXY=36.88759994506835937500
MINX=24.09291076660156250000
MAXX=24.71498107910156250000

generate_explicit() {
	local INPUT_GRAPH="$1"
	local OUTPUT_GRAPH="$2"
	local OUTPUT_GRAPH_GL="$3"
	local EPSILON="$4"
	make_explicit ${INPUT_GRAPH} ${OUTPUT_GRAPH} ${EPSILON}
	make_gl ${OUTPUT_GRAPH} ${OUTPUT_GRAPH_GL} 1 1
	project_graph ${OUTPUT_GRAPH_GL} ${OUTPUT_GRAPH_GL} xy_to_latlon
}


# project to latitude longitude
project_graph $FILE_IN_UNREF $FILE_IN_UNREF_PROJ xy_to_latlon
project_graph $FILE_IN_REF $FILE_IN_REF_PROJ xy_to_latlon

# prune refined and unrefined graphs
prune_graph -g $FILE_IN_UNREF_PROJ -o $FILE_UNREF -y $MINY -x $MINX -Y $MAXY -X $MAXX
prune_graph -g $FILE_IN_REF_PROJ -o $FILE_REF -y $MINY -x $MINX -Y $MAXY -X $MAXX

# make gl for pruned graphs
make_gl $FILE_REF $FILE_REF_GL 1 2
make_gl $FILE_UNREF $FILE_UNREF_GL 1 2

# project back
project_graph $FILE_UNREF $FILE_UNREF latlon_to_xy
project_graph $FILE_REF $FILE_REF latlon_to_xy

# make explicit
generate_explicit ${FILE_REF} ${FILE_REF_EXPLICIT_10} ${FILE_REF_EXPLICIT_GL_10} 1.0
generate_explicit ${FILE_REF} ${FILE_REF_EXPLICIT_05} ${FILE_REF_EXPLICIT_GL_05} 0.5

echo "pruned unrefined graph has size: "
head -n2 $FILE_UNREF

echo "pruned refined graph has size: "
head -n2 $FILE_REF
