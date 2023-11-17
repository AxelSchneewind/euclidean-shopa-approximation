#pragma once

#include "../graph/graph_impl.h"
#include "../triangulation/steiner_graph.h"
#include "formatters.h"


class triangulation_file_io {
public:
  template <Topology Graph, typename format = stream_encoders::encode_text>
  static Graph read (std::istream &input);

  static steiner_graph read_steiner(std::istream& input, float __epsilon);

    template <Topology Graph, typename format = stream_encoders::encode_text>
    static void write (std::ostream &output, const Graph& graph);
};

