#pragma once

#include "../graph/graph.h"
#include "formatters.h"

class gl_file_io {
public:
  template <typename node_info, typename edge_info, typename format = stream_encoders::encode_text>
  std::ostream& write (std::ostream &output, const graph<node_info, edge_info>& graph, int line_width = 2, int color = 2);
};
