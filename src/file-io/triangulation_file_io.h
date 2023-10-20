#pragma once

#include "../graph/graph_impl.h"
#include "formatters.h"


class triangulation_file_io {
public:
  template <typename node_info, typename edge_info, typename format = stream_encoders::encode_text>
  graph<node_info, edge_info> read (std::istream &input);
};
