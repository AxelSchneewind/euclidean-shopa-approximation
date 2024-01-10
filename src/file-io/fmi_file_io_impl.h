#pragma once

#include "fmi_file_io.h"
#include "file_io.h"
#include "formatters_impl.h"

#include <istream>
#include <ostream>
#include <span>
#include <vector>



template<typename Graph, typename Formatter>
Graph
fmi_file_io::read(std::istream &input_size, std::istream &input_nodes, std::istream &input_edges) {
    using f = Formatter;
    f::skip_comments(input_size);

    size_t node_count(f::template read<std::size_t>(input_size));
    size_t edge_count(f::template read<std::size_t>(input_size));

    std::vector<typename Graph::node_info_type> nodes(node_count);
    file_io::read_nodes<typename Graph::node_info_type, f>(input_nodes, nodes);
    std::vector<typename Graph::adjacency_list_type::builder::edge_type> edges(edge_count);
    file_io::read_edges<typename Graph::adjacency_list_type::builder::edge_type, f>(input_edges, edges);

    typename Graph::adjacency_list_type::builder builder;
    builder.add_edges(std::move(edges));
    builder.insert_backward_edges();

    auto list = Graph::adjacency_list_type::make_bidirectional_undirected(std::move(builder.get()));
    auto adj_list = typename Graph::adjacency_list_type(std::move(list));
    return Graph::make_graph(std::move(nodes), std::move(adj_list));
}

template<typename Graph, class formatter>
std::ostream &
fmi_file_io::write(std::ostream &output, const Graph &graph) {
    using f = formatter;

    node_id_t node_count = graph.node_count();
    node_id_t edge_count = graph.edge_count();

    f::write(output, node_count);
    f::write(output, '\n');
    f::write(output, edge_count);
    f::write(output, '\n');

    for (int n = 0; n < node_count; ++n) {
        f::write(output, graph.node(n));
        f::write(output, '\n');
    }

    for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++) {
        f::write(output, graph.topology().source(edge_index));
        f::write(output, ' ');
        f::write(output, graph.topology().destination(edge_index));
        f::write(output, ' ');
        f::write(output, graph.topology().edge(edge_index));
        f::write(output, '\n');
    }

    return output;
}
template<>
std::ostream &
fmi_file_io::write<steiner_graph, stream_encoders::encode_text>(std::ostream &output, const steiner_graph &graph) {
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
        f::write(output, n);
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
            f::write(output, n);
            f::write(output, '\n');
        }
    }

    // write all outgoing edges of base nodes
    for (auto node: graph.base_graph().node_ids()) {
        {
            auto &&edges = graph.outgoing_edges(node);
            for (auto edge: edges) {
                auto dest = edge.destination;
                assert(!graph.is_base_node(edge.destination));

                f::write(output, base_indices[node]) << ' ';
                f::write(output, indices[dest]) << ' ';
                f::write(output, distance(graph.node(node).coordinates, graph.node(dest).coordinates));
                f::write(output, '\n');
            }
        }
        {
            auto &&edges = graph.incoming_edges(node);
            for (auto edge: edges) {
                auto dest = edge.destination;
                assert(!graph.is_base_node(edge.destination));

                f::write(output, base_indices[node]) << ' ';
                f::write(output, indices[dest]) << ' ';
                f::write(output, distance(graph.node(node).coordinates, graph.node(dest).coordinates));
                f::write(output, '\n');
            }
        }
    }

    for (auto node: graph.node_ids()) {
        auto&& edges = graph.outgoing_edges(node);
        for (auto edge: edges) {
            auto dest = edge.destination;

            if (graph.is_base_node(node)) continue;

            // // avoid inserting an edge twice
            //if (indices[node] >= indices[dest] || !graph.has_edge(dest, node))
            //    continue;

            f::write(output, indices[node]) << ' ';
            f::write(output, indices[dest]) << ' ';
            f::write(output, distance(graph.node(node).coordinates, graph.node(dest).coordinates));
            f::write(output, "\n");
        }
    }

    return output;
}
