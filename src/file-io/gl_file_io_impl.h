#pragma once

#include "gl_file_io.h"

template <typename node_info, typename edge_info, typename format>
std::ostream& gl_file_io::write (std::ostream &output, const graph<node_info, edge_info>& graph, int line_width, int color)
{
  format f;

  node_id_t node_count = graph.node_count ();
  node_id_t edge_count = graph.edge_count ();

  f.write (output, node_count);
  f.write (output, '\n');
  f.write (output, edge_count);
  f.write (output, '\n');

  for (int n = 0; n < node_count; ++n)
    {
      f.write (output, graph.node (n).coordinates.latitude) << ' ';
      f.write (output, graph.node (n).coordinates.longitude);
      f.write(output, '\n');
    }

  for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++)
    {
      auto edge =  graph.forward().adjlist_edge (edge_index);

      // avoid inserting a edge twice
      if (edge.source >= edge.destination && graph.forward().has_edge (edge.destination, edge.source))
	continue;

      f.write (output, edge.source) << ' ';
      f.write (output, edge.destination) << ' ' << line_width << ' ' << color;
      f.write(output, '\n');
    }

  return output;
}
