#pragma once

#include "formatters.h"
#include "../graph/graph.h"

#include <istream>
#include <ostream>
#include <span>
#include <vector>

class fmi_file_io
{
public:
  template <typename Graph, typename format = stream_encoders::encode_text>
  static Graph read (std::istream &input);

  template <typename Graph, typename format = stream_encoders::encode_text>
  static std::ostream &write (std::ostream &output, const Graph &graph);
};
