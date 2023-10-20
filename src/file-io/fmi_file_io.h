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
  template <typename node_info, typename edge_info, typename format = stream_encoders::encode_text>
  graph<node_info, edge_info> read (std::istream &input);

  template <typename node_info, typename edge_info, typename format = stream_encoders::encode_text>
  std::ostream &write (std::ostream &output, const graph<node_info, edge_info> &graph);
};
