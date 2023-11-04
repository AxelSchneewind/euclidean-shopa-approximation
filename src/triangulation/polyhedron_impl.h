#pragma once

#include "polyhedron.h"

// can be generalized to any type of polyhedron (using variable instead of static sized arrays)
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace> polyhedron<BaseGraph, MaxNodesPerFace>::make_polyhedron(const BaseGraph &__base) {
    using node_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type;
    using edge_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::edge_id_type;
    const std::size_t edge_count = polyhedron<BaseGraph, MaxNodesPerFace>::edge_count;

    typename unidirectional_adjacency_list<node_id_type, std::array<edge_id_type, edge_count>>::adjacency_list_builder
            adjacent_edges_builder(__base.node_count());

    // iterate over triangulation edges
    std::vector<node_id_type> found_nodes;
    for (auto node1: __base.node_ids()) {
        for (auto t_edge: __base.outgoing_edges(node1)) {
            auto node2 = t_edge.destination;

            // find nodes on adjacent edges
            for (auto edge: __base.outgoing_edges(node1))
                if (found_nodes.size() < 40)
                    found_nodes.push_back(edge.destination);
            for (auto edge: __base.outgoing_edges(node2))
                if (found_nodes.size() < 40)
                    found_nodes.push_back(edge.destination);
            std::sort(found_nodes.begin(), found_nodes.end());

            // get duplicates from found nodes
            int index = 0;
            std::array<edge_id_type, edge_count> edges;
            for (int j = 1; j < found_nodes.size(); ++j) {
                if (found_nodes[j] == found_nodes[j - 1]) {
                    auto edge_id = __base.edge_id(node1, found_nodes[j]);
                    if (index == 0 || edge_id != edges[index - 1]) {
                        edges[index++] = edge_id;
                    }
                }
            }
            found_nodes.clear();

            // fill edges
            for (int j = index; j < edges.size(); ++j)
                edges[j] = NO_EDGE_ID;

            adjacent_edges_builder.add_edge(node1, node2, edges);
        }
    }

    auto first = adjacent_edges_builder.get();
    auto second = adjacency_list<node_id_type, std::array<node_id_type, edge_count>>::make_bidirectional_undirected(
            std::move(first));
    return polyhedron<BaseGraph, MaxNodesPerFace>(__base, second);
}
