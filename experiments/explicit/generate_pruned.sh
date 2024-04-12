#!/bin/bash

source ../utils.sh

# TRIANGLE_EXE=/opt/routing/graphs/aegaeis/triangle/triangle

FILE_IN_UNREF=$GRAPH_DIR/aegaeis/aegaeis-unref.graph
FILE_IN_REF=$GRAPH_DIR/aegaeis/aegaeis-ref-new.graph

FILE_IN_UNREF_PROJ=aegaeis-unref-latlon.graph
FILE_IN_REF_PROJ=aegaeis-ref-latlon.graph


# bounding box for milos island
MINY=36.50861358642578125000
MAXY=36.88759994506835937500
MINX=24.09291076660156250000
MAXX=24.71498107910156250000

# graph names for extracted triangulations
FILE_UNREF=milos.graph
FILE_REF=milos-ref.graph

# graph names for gl files for triangulations
FILE_UNREF_GL=milos.gl
FILE_REF_GL=milos-ref.gl

# filenames for explicit graph represenations
FILE_REF_EXPLICIT_10=milos-ref-10.fmi
FILE_REF_EXPLICIT_05=milos-ref-05.fmi
FILE_REF_EXPLICIT_025=milos-ref-025.fmi
FILE_UNREF_EXPLICIT=milos-unref.fmi

# filenames for gl files of explicit graph represenations (disabled for epsilon below 1)
FILE_REF_EXPLICIT_GL_10=milos-ref-explicit-10.gl
FILE_REF_EXPLICIT_GL_05=""
FILE_REF_EXPLICIT_GL_025=""
# FILE_REF_EXPLICIT_GL_05=milos-ref-explicit-05.gl
# FILE_REF_EXPLICIT_GL_025=milos-ref-explicit-025.gl


generate_explicit() {
	local INPUT_GRAPH="$1"
	local OUTPUT_GRAPH="$2"
	local OUTPUT_GRAPH_GL="$3"
	local EPSILON="$4"
	make_explicit ${INPUT_GRAPH} ${OUTPUT_GRAPH} ${EPSILON}

	if [ -n "$OUTPUT_GRAPH_GL" ]; then
		make_gl ${OUTPUT_GRAPH} ${OUTPUT_GRAPH_GL} 1 1
		project_graph ${OUTPUT_GRAPH_GL} ${OUTPUT_GRAPH_GL} xy_to_latlon
	fi

	echo "explicit graph has size (node count, edge count, lines): "
	head -n2 $OUTPUT_GRAPH
	wc -l $OUTPUT_GRAPH
	echo ""
}


# project to latitude/longitude
if [ ! -f "$FILE_IN_REF_PROJ" ]; then
	project_graph $FILE_IN_REF $FILE_IN_REF_PROJ xy_to_latlon
fi
if [ ! -f "$FILE_IN_UNREF_PROJ" ]; then
	project_graph $FILE_IN_UNREF $FILE_IN_UNREF_PROJ xy_to_latlon
fi


# prune refined and unrefined graphs
if [ ! -f "$FILE_REF" ]; then
	prune_graph -g $FILE_IN_REF_PROJ -o $FILE_REF -y $MINY -x $MINX -Y $MAXY -X $MAXX

	echo "pruned refined graph has size (node count, face count, lines): "
	head -n2 $FILE_REF
	wc -l $FILE_REF
	echo ""
fi
if [ ! -f "$FILE_UNREF" ]; then
	prune_graph -g $FILE_IN_UNREF_PROJ -o $FILE_UNREF -y $MINY -x $MINX -Y $MAXY -X $MAXX

	echo "pruned unrefined graph has size (node count, face count, lines): "
	head -n2 $FILE_UNREF
	wc -l $FILE_UNREF
	echo ""
fi


# make gl for pruned graphs and project back
if [ ! -f "$FILE_REF_GL" ]; then
	make_gl $FILE_REF $FILE_REF_GL 1 2
	project_graph $FILE_UNREF $FILE_UNREF latlon_to_xy
fi
if [ ! -f "$FILE_UNREF_GL" ]; then
	make_gl $FILE_UNREF $FILE_UNREF_GL 1 2
	project_graph $FILE_REF $FILE_REF latlon_to_xy
fi


# make explicit
generate_explicit ${FILE_REF} ${FILE_REF_EXPLICIT_10} ${FILE_REF_EXPLICIT_GL_10} 1.0
generate_explicit ${FILE_REF} ${FILE_REF_EXPLICIT_05} ${FILE_REF_EXPLICIT_GL_05} 0.5
generate_explicit ${FILE_REF} ${FILE_REF_EXPLICIT_025} ${FILE_REF_EXPLICIT_GL_025} 0.25
