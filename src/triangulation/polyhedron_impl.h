#pragma once

#include "polyhedron.h"

// can be generalized to any type of polyhedron (using variable instead of static sized arrays)
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::make_polyhedron(const BaseGraph &__base) {
    using node_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type;
    using edge_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::edge_id_type;
    const std::size_t edge_count = polyhedron<BaseGraph, MaxNodesPerFace>::edge_count;

    typename unidirectional_adjacency_list<node_id_type, std::array<edge_id_type, edge_count>>::adjacency_list_builder
            adjacent_edges_builder(__base.node_count());

    // iterate over triangulation edges
    std::vector<node_id_type> found_nodes;
    for (auto node1: __base.node_ids()) {
        for (auto base_edge: __base.outgoing_edges(node1)) {
            auto node2 = base_edge.destination;

            // find nodes on adjacent edges
            // TODO directly get edge ids
            for (auto edge: __base.incoming_edges(node1))
                found_nodes.push_back(edge.destination);
            for (auto edge: __base.incoming_edges(node2))
                found_nodes.push_back(edge.destination);
            for (auto edge: __base.outgoing_edges(node1))
                found_nodes.push_back(edge.destination);
            for (auto edge: __base.outgoing_edges(node2))
                found_nodes.push_back(edge.destination);
            std::sort(found_nodes.begin(), found_nodes.end());

            // get duplicates from found nodes
            std::vector<edge_id_type> found_edges;
            int index = 0;
            for (int j = 1; j < found_nodes.size(); ++j) {
                if (found_nodes[j] == found_nodes[j - 1]) {
                    auto edge_id1 = __base.edge_id(node1, found_nodes[j]);
                    auto edge_id2 = __base.edge_id(found_nodes[j], node1);
                    auto edge_id3 = __base.edge_id(node2, found_nodes[j]);
                    auto edge_id4 = __base.edge_id(found_nodes[j], node2);


                    if (edge_id1 >= 0 && edge_id1 < __base.edge_count())
                        found_edges.push_back(edge_id1);
                    if (edge_id2 >= 0 && edge_id2 < __base.edge_count())
                        found_edges.push_back(edge_id2);
                    if (edge_id3 >= 0 && edge_id3 < __base.edge_count())
                        found_edges.push_back(edge_id3);
                    if (edge_id4 >= 0 && edge_id4 < __base.edge_count())
                        found_edges.push_back(edge_id4);
                }
            }
            found_nodes.clear();
            std::sort(found_edges.begin(), found_edges.end());

            // store found edges in array
            index = 0;
            std::array<edge_id_type, edge_count> edges;
            for (int j = 0; j < found_edges.size(); ++j) {
                if (index == 0 || found_edges[j] != edges[index - 1]) {
                    assert(found_edges[j] >= 0 && found_edges[j] < __base.edge_count());
                    edges[index++] = found_edges[j];
                }
            }

            found_edges.clear();

            // fill edges
            for (int j = index; j < edges.size(); ++j)
                edges[j] = edge_id_type::NO_EDGE_ID;

            adjacent_edges_builder.add_edge(node1, node2, edges);
        }
    }

    auto first = adjacent_edges_builder.get();
    auto second = adjacency_list<node_id_type, std::array<edge_id_type, edge_count>>::make_bidirectional_undirected(
            std::move(first));
    return polyhedron<BaseGraph, MaxNodesPerFace>(__base, second);
}
