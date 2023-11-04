#pragma once

#include "fmi_file_io.h"

#include <istream>
#include <ostream>
#include <span>
#include <vector>

template<typename Graph, typename Formatter>
Graph
fmi_file_io::read(std::istream &__input) {
    using f = Formatter;
    f::skip_comments(__input);

    node_id_t node_count(f::template read<node_id_t>(__input));
    edge_id_t edge_count(f::template read<edge_id_t>(__input));

    std::vector<typename Graph::node_info_type> nodes(
            f::template read<typename Graph::node_info_type>(__input, node_count));

    typename unidirectional_adjacency_list<typename Graph::node_id_type, typename Graph::edge_info_type>::adjacency_list_builder builder(node_count);

    for (edge_id_t edge_index = 0; edge_index < edge_count; edge_index++)
        builder.add_edge(f::template read<adjacency_list_edge<typename Graph::node_id_type, typename Graph::edge_info_type>>(__input));

    auto unidirectional = std::move(builder).get();
    auto list = Graph::adjacency_list_type::make_bidirectional(std::move(unidirectional));
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
