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
void make_face_edges(const BaseGraph &base,
                     const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &faces,
                     std::vector<std::array<typename BaseGraph::edge_id_type, MaxNodesPerFace>> &face_edges,
                     std::vector<std::array<int, 2>> &edge_faces);

template<Topology BaseGraph, std::size_t MaxNodesPerFace>
void make_face_edges(const BaseGraph &base,
                     const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &faces,
                     std::vector<std::array<typename BaseGraph::edge_id_type, MaxNodesPerFace>> &face_edges,
                     std::vector<std::array<int, 2>> &edge_faces) {
    constexpr std::size_t edges_per_face = 3;

    face_edges.clear();
    face_edges.reserve(faces.size());

    edge_faces.clear();
    edge_faces.resize(base.edge_count());

    std::vector<char> triangle_count(base.edge_count());

    int face_index = 0;
    std::vector<typename BaseGraph::node_id_type> adjacent_edges;
    for (auto face: faces) {
        adjacent_edges.clear();

        for (int i = 0; i < MaxNodesPerFace; ++i) {
            auto node_id = face[i % MaxNodesPerFace];
            auto node_id_n = face[(i + 1) % MaxNodesPerFace];

            if (node_id > node_id_n)
                std::swap(node_id, node_id_n);
            if (node_id == node_id_n)
                continue;

            auto edge_id = base.edge_id(node_id, node_id_n);
            assert(!is_none(edge_id));
            adjacent_edges.emplace_back(edge_id);
        }

        std::sort(adjacent_edges.begin(), adjacent_edges.end());
        remove_duplicates_sorted(adjacent_edges);
        assert(adjacent_edges.size() == edges_per_face);

        // add edges array to face
        face_edges.emplace_back();
        int index = 0;
        for (auto e: adjacent_edges) {
            assert (base.source(e) < base.destination(e));
            face_edges.back().at(index++) = e;
        }
        assert(index <= MaxNodesPerFace);

        // add triangle to edges
        for (auto e: adjacent_edges) {
            edge_faces[e][triangle_count[e]++] = face_index;
        }

        face_index++;
    }

    // fill with -1 for edges that are only part of one face
    for (int i = 0; i < base.edge_count(); ++i) {
        assert(triangle_count[i] >= 1);
        assert(triangle_count[i] <= 2);
        while (triangle_count[i] < 2)
            edge_faces[i][triangle_count[i]++] = none_value<typename BaseGraph::edge_id_type>;
    }
}

template<Topology BaseGraph>
void make_inverse_edges(const BaseGraph &base, std::vector<typename BaseGraph::edge_id_type> &result) {
    result.clear();
    result.resize(base.edge_count());

    for (int i = 0; i < base.edge_count(); ++i) {
        auto src = base.source(i);
        auto dest = base.destination(i);
        result[i] = base.edge_id(dest, src);
    }
}


// can be generalized to any type of polyhedron (using variable instead of statically sized arrays)
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::make_polyhedron(const BaseGraph &base,
                                                        std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &&faces) {
    using node_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type;
    using edge_id_type = polyhedron<BaseGraph, MaxNodesPerFace>::edge_id_type;
    constexpr std::size_t face_count_per_edge = polyhedron<BaseGraph, MaxNodesPerFace>::FACE_COUNT_PER_EDGE;
    constexpr std::size_t edge_count_per_face = polyhedron<BaseGraph, MaxNodesPerFace>::EDGE_COUNT_PER_FACE;

    // get all adjacent edges for each face
    // and all adjacent edge_faces for each edge
    std::vector<std::array<edge_id_type, edge_count_per_face>> triangle_edges;
    std::vector<std::array<int, face_count_per_edge>> edge_triangles;
    make_face_edges(base, faces, triangle_edges, edge_triangles);

    // get the inverse edge for each edge
    std::vector<edge_id_type> inverse_edges;
    make_inverse_edges(base, inverse_edges);

    // make edges reachable from base nodes
    std::vector<edge_id_type> node_edges;
    std::vector<int> node_edge_offsets;
    {
        std::vector<steiner_graph::triangle_edge_id_type> adjacent_edge_ids;
        std::vector<steiner_graph::triangle_edge_id_type> triangle_edge_ids;
        for (std::size_t node = 0; node < base.node_count(); ++node) {
            // get edges and triangles adjacent to node
            for (auto&& edge: base.outgoing_edges(node)) {
                auto edge_id = base.edge_id(node, edge.destination);
                adjacent_edge_ids.push_back(edge_id);

                for (auto triangle: edge_triangles[edge_id]) {
                    if (is_none(triangle)) continue;
                    for (auto other_edge: triangle_edges[triangle])
                        triangle_edge_ids.push_back(other_edge);
                }
            }
            for (auto&& edge: base.incoming_edges(node)) {
                auto edge_id = base.edge_id(edge.destination, node);
                adjacent_edge_ids.push_back(edge_id);

                for (auto triangle: edge_triangles[edge_id]) {
                    if (is_none(triangle)) continue;
                    for (auto other_edge: triangle_edges[triangle])
                        triangle_edge_ids.push_back(other_edge);
                }
            }

            // assert(adjacent_edge_ids.size() >= 2);
            // assert(triangle_edge_ids.size() >= 6);

            std::sort(triangle_edge_ids.begin(), triangle_edge_ids.end());
            std::sort(adjacent_edge_ids.begin(), adjacent_edge_ids.end());

            // get edges that are only reachable via a triangle
            remove_duplicates_sorted(triangle_edge_ids);
            remove_duplicates_sorted(adjacent_edge_ids);
            int edge_count = set_minus_sorted<int>(triangle_edge_ids, adjacent_edge_ids, triangle_edge_ids);

            node_edge_offsets.push_back(node_edges.size());
            for (auto&& e: triangle_edge_ids)
                node_edges.emplace_back(e);

            adjacent_edge_ids.clear();
            triangle_edge_ids.clear();
        }

        node_edge_offsets.push_back(node_edges.size());
    }

    faces.clear();

    return polyhedron<BaseGraph, MaxNodesPerFace>(std::move(triangle_edges), std::move(edge_triangles),
                                                  std::move(inverse_edges), std::move(node_edges),
                                                  std::move(node_edge_offsets));
}

template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::polyhedron(
        std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>> &&adjacent_edges,
        std::vector<std::array<face_id_type, FACE_COUNT_PER_EDGE>> &&adjacent_faces,
        std::vector<edge_id_type> &&inverse_edges,
        std::vector<edge_id_type> &&node_edges,
        std::vector<int> &&node_face_offsets)
        : _M_face_info(std::move(adjacent_edges)),
          _M_edge_info(),
          _M_node_edges_offsets(std::move(node_face_offsets)),
          _M_node_edges(std::move(node_edges)) {

    int i = 0;
    while (!adjacent_faces.empty()) {
        _M_edge_info.emplace_back(adjacent_faces.back(), inverse_edges.back());

        adjacent_faces.pop_back();
        inverse_edges.pop_back();

        if ((i++) % (1024 * 1024) == 0) {
            adjacent_edges.shrink_to_fit();
            inverse_edges.shrink_to_fit();
        }
    }

    adjacent_faces.clear();
    inverse_edges.clear();

    list_invert(_M_edge_info);

    // check integrity
    std::size_t num_edges = 3 * face_count(); // 3 edges per face
    std::size_t non_boundary_edges = edge_count() - boundary_edge_count();

    // non boundary edges have been counted twice
    num_edges -= non_boundary_edges;

    assert(num_edges == edge_count());
}

