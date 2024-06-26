#pragma once

#include "formatters.h"
#include "../graph/graph.h"

#include <istream>
#include <ostream>
#include <span>
#include <vector>

class fmi_file_io {
public:
    template<typename Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input) { return read<Graph, format>(input, input, input); }

    template<typename Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input_sizes, std::istream &input_nodes, std::istream &input_edges);

    template<typename Graph, typename format = stream_encoders::encode_text>
    static std::ostream &write(std::ostream &output, const Graph &graph);

    template<SteinerGraph Graph, typename format = stream_encoders::encode_text>
    static std::ostream &write(std::ostream &output, const Graph &graph);
};
