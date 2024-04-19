#pragma once

#include "gl_file_io.h"
#include "formatters_impl.h"
#include "../triangulation/steiner_graph.h"

#include <iomanip>

template<typename Graph, typename format>
std::ostream &gl_file_io::write(std::ostream &output, const Graph &graph, int line_width, int color) {
    using f = format;

    size_t node_count = graph.node_count();
    size_t edge_count = graph.edge_count();

    f::write(output, node_count);
    f::write(output, '\n');
    f::write(output, edge_count);
    f::write(output, '\n');

    for (auto&& node: graph.node_ids()) {
        auto&& n = graph.node(node);
        f::write(output, n);
        f::write(output, '\n');
    }

    for (auto&& node: graph.node_ids()) {
        for (auto&& edge: graph.outgoing_edges(node)) {
            auto const& dest = edge.destination;

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

template<SteinerGraph Graph, class format>
std::ostream &
gl_file_io::write(std::ostream &output, const Graph &graph, int line_width, int color) {
    using f = stream_encoders::encode_text;

    size_t node_count = graph.node_count();
    size_t edge_count = graph.edge_count() / 2;

    f::write(output, node_count);
    f::write(output, '\n');
    f::write(output, edge_count);
    f::write(output, '\n');

    size_t index = 0;
    std::unordered_map<typename Graph::base_topology_type::node_id_type, size_t> base_indices;
    for (auto&& node: graph.base_graph().node_ids()) {
        base_indices[node] = index++;
        auto&& n = graph.node(node);
        f::write(output, n) << '\n';
    }

    std::unordered_map<typename Graph::node_id_type, size_t> indices;
    for (auto&& node: graph.node_ids()) {
        if (graph.is_base_node(node)) {
            indices[node] = base_indices[graph.base_node_id(node)];
        } else {
            indices[node] = index++;
            auto&& n = graph.node(node);
            f::write(output, n) << '\n';
        }
    }

    // write all outgoing edges of base nodes
    for (auto&& node: graph.base_graph().node_ids()) {
        auto &&edges = graph.outgoing_edges(node);
        for (auto&& edge: edges) {
            auto dest = edge.destination;

            // avoid inserting an edge twice
            if (base_indices[node] > indices[dest] && graph.has_edge(dest, graph.from_base_node_id(node)))
                continue;

            f::write(output, base_indices[node]) << ' ';
            f::write(output, indices[dest]) << ' ';

            // set line width and color depending on whether the edge is on a base edge or not
            if (graph.base_graph().source(dest.edge) == node || graph.base_graph().destination(dest.edge) == node)
                output << 2 * line_width << ' ' << '2';
            else
                output << line_width << ' ' << ((color == 2) ? 1 : color);

            f::write(output, '\n');
        }
    }

    // write remaining edges
    for (auto&& node: graph.node_ids()) {
        auto &&edges = graph.outgoing_edges(node);
        for (auto&& edge: edges) {
            auto dest = edge.destination;

            if (graph.is_base_node(node) || graph.is_base_node(dest)) continue;

            // avoid inserting an edge twice
            if (indices[node] > indices[dest] && graph.has_edge(dest, node))
                continue;

            f::write(output, indices[node]) << ' ';
            f::write(output, indices[dest]) << ' ';

            // set line width and color depending on whether the edge is on a base edge or not
            if (node.edge == dest.edge || graph.is_base_node(node) || graph.is_base_node(dest))
                output << 2 * line_width << ' ' << '2';
            else
                output << line_width << ' ' << ((color == 2) ? 1 : color);

            f::write(output, '\n');
        }
    }

    return output;
}
