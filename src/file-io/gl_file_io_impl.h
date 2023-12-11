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
std::ostream &
gl_file_io::write<steiner_graph>(std::ostream &output, const steiner_graph &graph, int line_width, int color) {
    using f = stream_encoders::encode_text;

    size_t node_count = graph.node_count();
    size_t edge_count = graph.edge_count();

    f::write(output, node_count);
    f::write(output, '\n');
    f::write(output, edge_count);
    f::write(output, '\n');

    size_t index = 0;
    std::unordered_map<steiner_graph::base_topology_type::node_id_type, size_t> base_indices;
    for (auto node: graph.base_graph().node_ids()) {
        base_indices[node] = index++;
        auto n = graph.node(node);
        f::write(output, n.coordinates.latitude) << ' ';
        f::write(output, n.coordinates.longitude);
        f::write(output, '\n');
    }

    std::unordered_map<steiner_graph::node_id_type, size_t> indices;
    for (auto node: graph.node_ids()) {
        if (node.steiner_index == 0) {
            indices[node] = base_indices[graph.base_graph().source(node.edge)];
        } else if (node.steiner_index == graph.steiner_info(node.edge).node_count - 1) {
            indices[node] = base_indices[graph.base_graph().destination(node.edge)];
        } else {
            indices[node] = index++;
            auto n = graph.node(node);
            f::write(output, n.coordinates.latitude) << ' ';
            f::write(output, n.coordinates.longitude);
            f::write(output, '\n');
        }
    }

    // write all outgoing edges of base nodes
    for (auto node: graph.base_graph().node_ids()) {
        auto &&edges = graph.outgoing_edges(node);
        for (auto edge: edges) {
            auto dest = edge.destination;
            assert(!graph.is_base_node(edge.destination));

            f::write(output, base_indices[node]) << ' ';
            f::write(output, indices[dest]) << ' ';

            // set line width and color depending on whether the edge is a base edge or not
            if (graph.base_graph().source(dest.edge) == node || graph.base_graph().destination(dest.edge) == node)
                output << 2 * line_width << ' ' << '2';
            else
                output << line_width << ' ' << ((color == 2) ? 1 : color);

            f::write(output, '\n');
        }

    }

    for (auto node: graph.node_ids()) {
        auto &&edges = graph.outgoing_edges(node);
        for (auto edge: edges) {
            auto dest = edge.destination;

            if (graph.is_base_node(node) || graph.is_base_node(dest)) continue;

            // avoid inserting an edge twice
            if (indices[node] >= indices[dest] || !graph.has_edge(dest, node))
                continue;

            f::write(output, indices[node]) << ' ';
            f::write(output, indices[dest]) << ' ';

            // set line width and color depending on whether the edge is a base edge or not
            if (node.edge == dest.edge || graph.is_base_node(node) || graph.is_base_node(dest))
                output << 2 * line_width << ' ' << '2';
            else
                output << line_width << ' ' << ((color == 2) ? 1 : color);

            f::write(output, '\n');
        }
    }

    return output;
}
