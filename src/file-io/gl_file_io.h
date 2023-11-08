#pragma once

#include "../graph/graph.h"
#include "formatters.h"
#include "../triangulation/steiner_graph.h"

class gl_file_io {
public:
    template<Topology Graph, typename format = stream_encoders::encode_text>
    static std::ostream &write(std::ostream &output, const Graph &graph, int line_width = 2, int color = 2);
};
