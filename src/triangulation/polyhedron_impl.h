#pragma once

#include "polyhedron.h"
#include "../util/keep_duplicates.h"
#include "../util/list_intersection.h"
#include "../util/remove_duplicates.h"
#include "../util/debug.h"

template<typename T, size_t Cap>
class fixed_capacity_vector {
private:
    short i;
    std::array<T, Cap> _M_data;
public:
    fixed_capacity_vector() : i(0) {}

    void push_back(T element) {
        assert(i < Cap);
        _M_data[i++] = element;
    }

    size_t size() const { return i; }

    auto begin() { return _M_data.begin(); }

    auto end() { return _M_data.begin() + i; }
};

// can be generalized to any type of polyhedron (using variable instead of static sized arrays)
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::make_polyhedron(const BaseGraph &__base,
                                                        const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &faces) {
    using node_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type;
    using edge_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::edge_id_type;
    const std::size_t edge_count_per_edge = polyhedron<BaseGraph, MaxNodesPerFace>::EDGE_COUNT;

    // for each edge, store the triangles it belongs to
    size_t const edge_count = __base.edge_count();
    size_t const triangle_index_max = edge_count - 1;
    std::vector<fixed_capacity_vector<int, 2>> triangles(edge_count);
    int face_index = 0;
    for (auto face: faces) {
        // keep track of the triangles each node is part of
        auto node_id = face[0];
        auto node_id_n1 = face[1];
        auto node_id_n2 = face[2];
        assert(node_id != node_id_n1 && node_id != node_id_n2 && node_id_n1 != node_id_n2);

        triangles[triangle_index_max - __base.edge_id(node_id, node_id_n1)].push_back(face_index);
        triangles[triangle_index_max - __base.edge_id(node_id, node_id_n2)].push_back(face_index);
        triangles[triangle_index_max - __base.edge_id(node_id_n1, node_id_n2)].push_back(face_index);
        triangles[triangle_index_max - __base.edge_id(node_id_n1, node_id)].push_back(face_index);
        triangles[triangle_index_max - __base.edge_id(node_id_n2, node_id_n1)].push_back(face_index);
        triangles[triangle_index_max - __base.edge_id(node_id_n2, node_id)].push_back(face_index);

        face_index++;
    }

    // sort all lists and remove duplicates
    for (auto base_node: __base.node_ids()) {
        for (auto base_edge: __base.outgoing_edges(base_node)) {
            std::sort(triangles[__base.edge_id(base_node, base_edge.destination)].begin(),
                      triangles[__base.edge_id(base_node, base_edge.destination)].end());
        }
    }

    // iterate over triangulation edges
    std::vector<std::array<edge_id_type, edge_count_per_edge>> result;
    std::vector<edge_id_t> adjacent_edges;
    int counter = 0;
    for (size_t base_edge_id = 0; base_edge_id < edge_count; base_edge_id++) {
        auto base_node_1 = __base.source(base_edge_id);
        auto base_node_2 = __base.destination(base_edge_id);
        auto base_edge_id_inv = __base.edge_id(base_node_2, base_node_1);

        // get faces adjacent to both nodes
        //assert(!triangles[base_edge_id].empty() && triangles[base_edge_id].size() <= 2);
        assert(triangles.size() == edge_count - base_edge_id);
        assert(triangles[triangle_index_max - base_edge_id].size() >= 1);
        for (auto face_index: triangles[triangle_index_max - base_edge_id]) {
            auto face = faces[face_index];
            int t = 0;
            while (face[t] != base_node_1) t++;

            auto node_id = face[t];
            auto node_id_n1 = face[(t + 1) % 3];
            auto node_id_n2 = face[(t + 2) % 3];

            // TODO only use edges that base_node_1 is part of
            adjacent_edges.push_back(__base.edge_id(node_id, node_id_n1));
            adjacent_edges.push_back(__base.edge_id(node_id, node_id_n2));
            adjacent_edges.push_back(__base.edge_id(node_id_n1, node_id));
            /*adjacent_edges.push_back(__base.edge_id(node_id_n1, node_id_n2));*/
            adjacent_edges.push_back(__base.edge_id(node_id_n2, node_id));
            /*adjacent_edges.push_back(__base.edge_id(node_id_n2, node_id_n1));*/
        }
        triangles.pop_back();
        if ((counter++) % (1024 * 1024) == 0)
            triangles.shrink_to_fit();

        std::sort(adjacent_edges.begin(), adjacent_edges.end());
        remove_duplicates_sorted(adjacent_edges);
        assert(adjacent_edges.size() >= 4);
        assert(adjacent_edges.size() <= edge_count_per_edge + 1);

        // make array and attach it to edge
        // own edge is not present, inverse edge id is first entry, others are the remaining ones (in ascending order)
        result.emplace_back();
        result.back().at(0) = base_edge_id_inv;
        int index = 1;
        for (auto e: adjacent_edges) {
            if (e != base_edge_id && e != base_edge_id_inv && index < edge_count_per_edge)
                result.back().at(index++) = e;
        }
        while (index < edge_count_per_edge) {
            result.back().at(index++) = none_value<edge_id_type>();
        }

        adjacent_edges.clear();
    }

    return polyhedron<BaseGraph, MaxNodesPerFace>(std::move(result));
}
