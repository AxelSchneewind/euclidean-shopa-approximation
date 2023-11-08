#pragma once

#include "polyhedron.h"
#include "../util/keep_duplicates.h"
#include "../util/list_intersection.h"
#include "../util/remove_duplicates.h"

// can be generalized to any type of polyhedron (using variable instead of static sized arrays)
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::make_polyhedron(const BaseGraph &__base,
                                                        const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &faces) {
    using node_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type;
    using edge_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::edge_id_type;
    const std::size_t edge_count = polyhedron<BaseGraph, MaxNodesPerFace>::edge_count;

    typename unidirectional_adjacency_list<node_id_type, std::array<edge_id_type, edge_count>>::adjacency_list_builder
            adjacent_edges_builder(__base.node_count());

    // for each edge, store the triangles it belongs to
    std::vector<std::vector<int>> triangles(__base.edge_count());
    int face_index = 0;
    for (auto face: faces) {
        // keep track of the triangles each node is part of
        auto node_id = face[0];
        auto node_id_n1 = face[1];
        auto node_id_n2 = face[2];
        assert(node_id != node_id_n1 && node_id != node_id_n2 && node_id_n1 != node_id_n2);

        triangles[__base.edge_id(node_id, node_id_n1)].push_back(face_index);
        triangles[__base.edge_id(node_id, node_id_n2)].push_back(face_index);
        triangles[__base.edge_id(node_id_n1, node_id_n2)].push_back(face_index);
        triangles[__base.edge_id(node_id_n1, node_id)].push_back(face_index);
        triangles[__base.edge_id(node_id_n2, node_id_n1)].push_back(face_index);
        triangles[__base.edge_id(node_id_n2, node_id)].push_back(face_index);

        face_index++;
    }

    // sort all lists and remove duplicates
    for (auto base_node: __base.node_ids()) {
        for (auto base_edge: __base.outgoing_edges(base_node)) {
            std::sort(triangles[__base.edge_id(base_node, base_edge.destination)].begin(),
                      triangles[__base.edge_id(base_node, base_edge.destination)].end());
            remove_duplicates_sorted(triangles[__base.edge_id(base_node, base_edge.destination)]);
            assert(triangles[__base.edge_id(base_node, base_edge.destination)].size() <= 2);
        }
    }

    // iterate over triangulation edges
    for (auto base_node_1: __base.node_ids()) {
        for (auto base_edge: __base.outgoing_edges(base_node_1)) {
            auto base_node_2 = base_edge.destination;
            auto base_edge_id = __base.edge_id(base_node_1, base_node_2);

            // get faces adjacent to both nodes
            assert(!triangles[base_edge_id].empty() && triangles[base_edge_id].size() <= 2);
            std::vector<edge_id_t> adjacent_edges;
            for (auto face_index: triangles[base_edge_id]) {
                auto face = faces[face_index];
                auto node_id = face[0];
                auto node_id_n1 = face[(0 + 1) % 3];
                auto node_id_n2 = face[(0 + 2) % 3];

                adjacent_edges.push_back(__base.edge_id(node_id, node_id_n1));
                adjacent_edges.push_back(__base.edge_id(node_id, node_id_n2));
                adjacent_edges.push_back(__base.edge_id(node_id_n1, node_id));
                adjacent_edges.push_back(__base.edge_id(node_id_n1, node_id_n2));
                adjacent_edges.push_back(__base.edge_id(node_id_n2, node_id));
                adjacent_edges.push_back(__base.edge_id(node_id_n2, node_id_n1));
            }
            std::sort(adjacent_edges.begin(), adjacent_edges.end());
            remove_duplicates_sorted(adjacent_edges);
            assert(adjacent_edges.size() >= 6);
            assert(adjacent_edges.size() <= edge_count);

            // make array and attach it to edge
            std::array<edge_id_type, edge_count> edges{};
            int index = 0;
            for (auto e: adjacent_edges) {
                if (index < edge_count)
                    edges[index++] = e;
            }
            while (index < edge_count) {
                edges[index++] = none_value<edge_id_type>();
            }

            adjacent_edges_builder.add_edge(base_node_1, base_node_2, edges);
        }
    }

     auto first = adjacent_edges_builder.get();
    auto second = adjacency_list<node_id_type, std::array<edge_id_type, edge_count>>::make_bidirectional_undirected(
            std::move(first));

    return polyhedron<BaseGraph, MaxNodesPerFace>(__base, second);
}
