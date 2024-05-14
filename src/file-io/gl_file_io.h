#pragma once

#include "../graph/graph.h"
#include "formatters.h"
#include "../triangulation/steiner_graph.h"
#include "fmi_file_io.h"

class gl_file_io {
public:
    template<typename Graph, typename format = stream_encoders::encode_text>
    static std::ostream &write(std::ostream &output, const Graph &graph, int line_width = 1, int color = 2);

    template<typename Graph, typename format = stream_encoders::encode_text>
    static Graph read(std::istream &input) { return fmi_file_io::read<Graph, format>(input, input, input); }

    template<SteinerGraph Graph, typename format = stream_encoders::encode_text>
    static std::ostream &write(std::ostream &output, const Graph &graph, int line_width = 1, int color = 2);
};
