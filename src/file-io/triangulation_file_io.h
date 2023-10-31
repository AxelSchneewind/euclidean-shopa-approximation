#pragma once

#include "../graph/graph_impl.h"
#include "formatters.h"


class triangulation_file_io {
public:
  template <typename Graph, typename format = stream_encoders::encode_text>
  Graph read (std::istream &input);
};
