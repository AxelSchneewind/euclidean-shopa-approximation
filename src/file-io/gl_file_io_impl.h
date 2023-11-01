#pragma once

#include "gl_file_io.h"

template<RoutableGraph Graph, typename format>
std::ostream &gl_file_io::write(std::ostream &output, const Graph &graph, int line_width, int color) {
    format f;

    node_id_t node_count = graph.node_count();
    node_id_t edge_count = graph.edge_count();

    f.write(output, node_count);
    f.write(output, '\n');
    f.write(output, edge_count);
    f.write(output, '\n');

    for (int n = 0; n < node_count; ++n) {
        f.write(output, graph.node(n).coordinates.latitude) << ' ';
        f.write(output, graph.node(n).coordinates.longitude);
        f.write(output, '\n');
    }

    for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++) {
        auto src = graph.topology().source(edge_index);
        auto dest = graph.topology().destination(edge_index);

        // avoid inserting an edge twice
        if (src >= dest && graph.topology().has_edge(dest, src))
            continue;

        f.write(output, src) << ' ';
        f.write(output, dest) << ' ' << line_width << ' ' << color;
        f.write(output, '\n');
    }

    return output;
}
