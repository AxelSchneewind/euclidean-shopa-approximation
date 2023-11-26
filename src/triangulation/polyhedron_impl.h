#pragma once

#include "polyhedron.h"

#include "../util/keep_duplicates.h"
#include "../util/list_intersection.h"
#include "../util/list_invert.h"
#include "../util/remove_duplicates.h"

template<typename T, size_t Cap>
class fixed_capacity_vector {
private:
    unsigned short i;
    std::array<T, Cap> _M_data;
public:
    fixed_capacity_vector() : i(0), _M_data() {}

    void push_back(T element) {
        assert(i < Cap);
        _M_data[i++] = element;
    }

    T &operator[](size_t index) { return _M_data[index]; }

    T operator[](size_t index) const { return _M_data[index]; }

    size_t size() const { return i; }

    auto begin() { return _M_data.begin(); }

    auto end() { return _M_data.begin() + i; }

    operator std::array<T, Cap>() {
        return _M_data;
    }

    void clear() { i = 0; }
};


template<Topology BaseGraph, std::size_t MaxNodesPerFace>
void make_face_edges(const BaseGraph &__base,
                     const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &__faces,
                     std::vector<std::array<typename BaseGraph::edge_id_type, 2 * (MaxNodesPerFace)>> &__face_edges,
                     std::vector<std::array<int, 2>> &__edge_faces) {
    constexpr size_t edges_per_face = 6;

    __face_edges.clear();
    __face_edges.reserve(__faces.size());

    __edge_faces.clear();
    __edge_faces.resize(__base.edge_count());

    std::vector<char> triangle_count(__base.edge_count());

    int face_index = 0;
    std::vector<typename BaseGraph::node_id_type> adjacent_edges;
    for (auto face: __faces) {
        adjacent_edges.clear();

        for (int i = 0; i < MaxNodesPerFace; ++i) {
            auto node_id = face[i % MaxNodesPerFace];
            auto node_id_n = face[(i + 1) % MaxNodesPerFace];

            adjacent_edges.push_back(__base.edge_id(node_id, node_id_n));
            adjacent_edges.push_back(__base.edge_id(node_id_n, node_id));
        }

        std::sort(adjacent_edges.begin(), adjacent_edges.end());
        remove_duplicates_sorted(adjacent_edges);
        assert(adjacent_edges.size() == edges_per_face);

        // add edges array
        __face_edges.emplace_back();
        int index = 0;
        for (auto e: adjacent_edges) {
            __face_edges.back().at(index++) = e;
        }

        // add triangle to edges
        for (auto e: adjacent_edges) {
            __edge_faces[e][triangle_count[e]++] = face_index;
        }

        face_index++;
    }

    // fill with -1 for edges that are only part of one face
    for (int i = 0; i < __base.edge_count(); ++i) {
        assert(triangle_count[i] >= 1);
        assert(triangle_count[i] <= 2);
        if (triangle_count[i] < 2)
            __edge_faces[i][triangle_count[i]++] = none_value<typename BaseGraph::edge_id_type>;
    }
}

template<Topology BaseGraph>
void make_inverse_edges(const BaseGraph &__base, std::vector<typename BaseGraph::edge_id_type> &__result) {
    __result.clear();
    __result.resize(__base.edge_count());

    for (int i = 0; i < __base.edge_count(); ++i) {
        auto src = __base.source(i);
        auto dest = __base.destination(i);
        __result[i] = __base.edge_id(dest, src);
    }
}


// can be generalized to any type of polyhedron (using variable instead of static sized arrays)
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::make_polyhedron(const BaseGraph &__base,
                                                        const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &faces) {
    using node_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type;
    using edge_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::edge_id_type;
    constexpr std::size_t face_count_per_edge = polyhedron<BaseGraph, MaxNodesPerFace>::FACE_COUNT_PER_EDGE;
    constexpr std::size_t edge_count_per_face = polyhedron<BaseGraph, MaxNodesPerFace>::EDGE_COUNT_PER_FACE;

    // get all adjacent edges for each face
    // and all adjacent edge_faces for each edge
    std::vector<std::array<edge_id_type, edge_count_per_face>> triangle_edges;
    std::vector<std::array<int, face_count_per_edge>> edge_triangles;
    make_face_edges(__base, faces, triangle_edges, edge_triangles);

    // get the inverse edge for each edge
    std::vector<edge_id_type> inverse_edges;
    make_inverse_edges(__base, inverse_edges);

    return polyhedron<BaseGraph, MaxNodesPerFace>(std::move(triangle_edges), std::move(edge_triangles),
                                                  std::move(inverse_edges));
}

template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::polyhedron(
        std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>> &&__adjacent_edges,
        std::vector<std::array<face_id_type, FACE_COUNT_PER_EDGE>> &&__adjacent_faces,
        std::vector<edge_id_type> &&__inverse_edges)
        : _M_face_info(std::move(__adjacent_edges)),
          _M_edge_info() {
    int i = 0;
    while (!__adjacent_faces.empty()) {
        edge_info_type info = {__adjacent_faces.back(), __inverse_edges.back()};
        _M_edge_info.push_back(info);

        __adjacent_faces.pop_back();
        __inverse_edges.pop_back();

        if ((i++) % (1024 * 1024) == 0) {
            __adjacent_edges.shrink_to_fit();
            __inverse_edges.shrink_to_fit();
        }
    }

    __adjacent_faces.clear();
    __inverse_edges.clear();

    list_invert(_M_edge_info);
}

