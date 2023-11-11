#pragma once

#include "gl_file_io.h"
#include "../triangulation/steiner_graph.h"

template<Topology Graph, typename format>
std::ostream &gl_file_io::write(std::ostream &output, const Graph &graph, int line_width, int color) {
    using f = format;

    size_t node_count = graph.node_count();
    size_t edge_count = graph.edge_count();

    f::write(output, node_count);
    f::write(output, '\n');
    f::write(output, edge_count);
    f::write(output, '\n');

    for (auto node: graph.node_ids()) {
        auto n = graph.node(node);
        f::write(output, n.coordinates.latitude) << ' ';
        f::write(output, n.coordinates.longitude);
        f::write(output, '\n');
    }

    for (auto node: graph.node_ids()) {
        for (auto edge: graph.outgoing_edges(node)) {
            auto dest = edge.destination;

            // avoid inserting an edge twice
            if (node >= dest && graph.has_edge(dest, node))
                continue;

            f::write(output, node) << ' ';
            f::write(output, dest) << ' ' << line_width << ' ' << color;
            f::write(output, '\n');
        }
    }

    return output;
}

template<>
std::ostream &gl_file_io::write<steiner_graph>(std::ostream &output, const steiner_graph &graph, int line_width, int color) {
    using f = stream_encoders::encode_text;

    size_t node_count = graph.node_count();
    size_t edge_count = graph.edge_count();

    f::write(output, node_count);
    f::write(output, '\n');
    f::write(output, edge_count);
    f::write(output, '\n');

    std::unordered_map<steiner_graph::node_id_type, size_t> indices;
    size_t index = 0;
    for (auto node: graph.node_ids()) {
        indices[node] = index++;
        auto n = graph.node(node);
        f::write(output, n.coordinates.latitude) << ' ';
        f::write(output, n.coordinates.longitude);
        f::write(output, '\n');
    }

    for (auto node: graph.node_ids()) {
        for (auto edge: graph.outgoing_edges(node)) {
            auto dest = edge.destination;

            // avoid inserting an edge twice
            if (indices[node] >= indices[dest] && graph.has_edge(dest, node))
                continue;

            f::write(output, indices[node]) << ' ';
            f::write(output, indices[dest]) << ' ' << line_width << ' ' << color;
            f::write(output, '\n');
        }
    }

    return output;
}
