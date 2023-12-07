#pragma once

#include "fmi_file_io.h"

#include <istream>
#include <ostream>
#include <span>
#include <vector>


template<typename NodeInfo, typename formatter>
std::vector<NodeInfo>
fmi_file_io::read_nodes(std::istream &input, std::size_t count) {
    std::vector<NodeInfo> nodes;
    for (int i = 0; i < count; ++i) {
        NodeInfo n;
        n.coordinates = formatter::template read<coordinate_t>(input);
        nodes.push_back(n);
    }
    return nodes;
}

template<typename NodeInfo, typename NodeId, typename EdgeInfo, typename formatter>
auto
fmi_file_io::read_edges(std::istream &input, std::vector<NodeInfo> const &nodes, std::size_t count) {
    typename unidirectional_adjacency_list<NodeId, EdgeInfo>::adjacency_list_builder builder(nodes.size());

    for (edge_id_t edge_index = 0; edge_index < count; edge_index++)
        builder.add_edge(formatter::template read<adjacency_list_edge<NodeId, EdgeInfo>>(input));

    return builder;
}


template<typename Graph, typename Formatter>
Graph
fmi_file_io::read(std::istream &input_size, std::istream &input_nodes, std::istream &input_edges) {
    using f = Formatter;
    f::skip_comments(input_size);

    size_t node_count(f::template read<std::size_t>(input_size));
    size_t edge_count(f::template read<std::size_t>(input_size));

    std::vector<typename Graph::node_info_type> nodes = read_nodes<typename Graph::node_info_type, f>(input_nodes,
                                                                                                      node_count);
    auto edges = read_edges<typename Graph::node_info_type, typename Graph::node_id_type, typename Graph::edge_info_type, f>(
            input_edges, nodes, edge_count);

    auto list = Graph::adjacency_list_type::make_bidirectional(std::move(edges.get()));
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
