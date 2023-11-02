#pragma once

#include "triangulation_file_io.h"

#include "../graph/graph_impl.h"
#include "../graph/adjacency_list_impl.h"
#include "../graph/geometry.h"

#include "formatters_impl.h"

template<Topology Graph, typename formatter>
Graph
triangulation_file_io::read(std::istream &input) {
    using f = formatter;
    f::skip_comments(input);

    node_id_t node_count(f::template read<node_id_t>(input));
    edge_id_t edge_count(f::template read<edge_id_t>(input));

    // read nodes
    std::vector<typename Graph::node_info_type> nodes;
    for (int i = 0; i < node_count; ++i) {
        node_t n;
        n.coordinates = f::template read<coordinate_t>(input);
        nodes.push_back(n);
    }

    // build adjacency list
    typename unidirectional_adjacency_list<node_id_t, edge_t>::adjacency_list_builder builder;
    builder.add_node(node_count - 1);

    // read triangles and generate edges from them
    for (int t = 0; t < edge_count; t++) {
        triangle tri = f::template read<triangle>(input);
        for (int i = 0; i < 3; ++i) {
            auto next = (i + 1) % 3;
            edge_t edge;
            edge.cost = distance(nodes[tri[i]].coordinates, nodes[tri[next]].coordinates);

            builder.add_edge(tri[i], tri[next], edge);
            builder.add_edge(tri[next], tri[i], edge);
        }
    }

    auto adj_list = Graph::adjacency_list_type::make_bidirectional_undirected(std::move(builder).get());
    return Graph::make_graph(std::move(nodes), std::move(adj_list));
}
