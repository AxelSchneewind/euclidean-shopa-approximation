#pragma once

#include "../graph/graph.h"
#include "../triangulation/steiner_graph.h"
#include "formatters.h"


class triangulation_file_io {
public:
    template<Topology Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input) { return read<Graph, format> (input, input, input); }
    template<Topology Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input_size, std::istream& input_nodes, std::istream& input_edges);

    static steiner_graph read_steiner(std::istream &input_size, std::istream& input_nodes, std::istream& input_triangles, double epsilon);
    static steiner_graph read_steiner(std::istream &input, double epsilon) { return read_steiner(input, input, input, epsilon); };

    template<Topology Graph, typename format = stream_encoders::encode_text>
    static void write(std::ostream &output, const Graph &graph);
};

