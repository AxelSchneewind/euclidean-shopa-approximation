#pragma once

#include "../routing/dijkstra_concepts.h"
#include "polyhedron.h"

#include "../util/list_intersection.h"
#include "../util/list_invert.h"

#include <array>
#include <vector>

namespace {
    template<Topology BaseGraph, std::size_t MaxNodesPerFace>
    void make_face_edges(const BaseGraph&base,
                         const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>>&faces,
                         std::vector<std::array<typename BaseGraph::edge_id_type, MaxNodesPerFace>>&face_edges,
                         std::vector<std::array<int, 2>>&edge_faces);

    template<Topology BaseGraph, std::size_t MaxNodesPerFace>
    void make_face_edges(const BaseGraph&base,
                         const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>>&faces,
                         std::vector<std::array<typename BaseGraph::edge_id_type, MaxNodesPerFace>>&face_edges,
                         std::vector<std::array<int, 2>>&edge_faces) {
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
                assert(base.source(e) < base.destination(e));
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
            // assert(triangle_count[i] >= 1);
            assert(triangle_count[i] <= 2);
            while (triangle_count[i] < 2)
                edge_faces[i][triangle_count[i]++] = none_value<typename BaseGraph::edge_id_type>;
        }
    }

    template<Topology BaseGraph>
    void make_inverse_edges(const BaseGraph&base, std::vector<typename BaseGraph::edge_id_type>&result) {
        result.clear();
        result.resize(base.edge_count());

        for (int i = 0; i < base.edge_count(); ++i) {
            auto src = base.source(i);
            auto dest = base.destination(i);
            result[i] = base.edge_id(dest, src);
        }
    }

    template<typename BaseTopology, typename FaceId, size_t FaceCountPerEdge>
    void make_boundary_nodes(const BaseTopology &triangulation,
                         std::vector<std::array<FaceId, FaceCountPerEdge>> const& edge_faces, std::vector<bool> &out) {
        out.resize(triangulation.node_count(), false);

        // a vertex is a boundary vertex if there exists an edge that only has one adjacent face
        for (int i = 0; i < triangulation.edge_count(); i++) {
            auto&& src = triangulation.source(i);
            auto&& dest = triangulation.destination(i);
            bool one_adjacent_face = is_none(edge_faces[i][1]);
            out[src] = out[src] || one_adjacent_face;
            out[dest] = out[dest] || one_adjacent_face;
        }
    }
    template<typename BaseTopology, typename FaceId, size_t FaceCountPerEdge>
    void make_boundary_edges(const BaseTopology &triangulation,
                         std::vector<std::array<FaceId, FaceCountPerEdge>> const& edge_faces, std::vector<bool> &out) {
        out.resize(triangulation.edge_count(), false);

        // a vertex is a boundary vertex if there exists an edge that only has one adjacent face
        for (int i = 0; i < triangulation.edge_count(); i++) {
            const bool one_adjacent_face = is_none(edge_faces[i][1]);
            out[i] = one_adjacent_face;
        }
    }
}

// can be generalized to any type of polyhedron (using variable instead of statically sized arrays)
template<Topology BaseGraph, size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace> polyhedron<BaseGraph, MaxNodesPerFace>::make_polyhedron(BaseGraph const &triangulation_edges,
                                    std::vector<std::array<typename polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type, MaxNodesPerFace>> &&faces){
    using node_id_type = typename polyhedron<BaseGraph, MaxNodesPerFace>::node_id_type;
    using edge_id_type = typename polyhedron<BaseGraph, MaxNodesPerFace>::edge_id_type;
    using face_id_type = typename polyhedron<BaseGraph, MaxNodesPerFace>::face_id_type;
    constexpr std::size_t face_count_per_edge = polyhedron<BaseGraph, MaxNodesPerFace>::FACE_COUNT_PER_EDGE;
    constexpr std::size_t edge_count_per_face = polyhedron<BaseGraph, MaxNodesPerFace>::EDGE_COUNT_PER_FACE;

    // get all adjacent edges for each face
    // and all adjacent edge_faces for each edge
    std::vector<std::array<edge_id_type, edge_count_per_face>> face_edges;
    std::vector<std::array<face_id_type, face_count_per_edge>> edge_faces;
    make_face_edges(triangulation_edges, faces, face_edges, edge_faces);

    // make edges reachable from base nodes
    std::vector<edge_id_type> node_edges;
    std::vector<int> node_edge_offsets; {
        std::vector<steiner_graph::triangle_edge_id_type> adjacent_edge_ids;
        std::vector<steiner_graph::triangle_edge_id_type> triangle_edge_ids;
        for (std::size_t node = 0; node < triangulation_edges.node_count(); ++node) {
            // get edges and triangles adjacent to node
            for (auto&&edge: triangulation_edges.outgoing_edges(node)) {
                auto edge_id = triangulation_edges.edge_id(node, edge.destination);
                assert(!triangulation_edges.has_edge(edge.destination, node));
                adjacent_edge_ids.push_back(edge_id);

                for (auto triangle: edge_faces[edge_id]) {
                    if (is_none(triangle)) continue;
                    for (auto other_edge: face_edges[triangle])
                        triangle_edge_ids.push_back(other_edge);
                }
            }
            for (auto&&edge: triangulation_edges.incoming_edges(node)) {
                auto edge_id = triangulation_edges.edge_id(edge.destination, node);
                assert(!triangulation_edges.has_edge(node, edge.destination));
                adjacent_edge_ids.push_back(edge_id);

                for (auto triangle: edge_faces[edge_id]) {
                    if (is_none(triangle)) continue;
                    for (auto other_edge: face_edges[triangle])
                        triangle_edge_ids.push_back(other_edge);
                }
            }

            std::sort(triangle_edge_ids.begin(), triangle_edge_ids.end());
            std::sort(adjacent_edge_ids.begin(), adjacent_edge_ids.end());

            // get edges that are only reachable via a triangle
            remove_duplicates_sorted(triangle_edge_ids);
            remove_duplicates_sorted(adjacent_edge_ids);
            int edge_count = 0;
            if (!triangle_edge_ids.empty())
                set_minus_sorted<int>(triangle_edge_ids, adjacent_edge_ids, triangle_edge_ids);

            node_edge_offsets.push_back(node_edges.size());
            for (auto&&e: triangle_edge_ids)
                node_edges.emplace_back(e);

            adjacent_edge_ids.clear();
            triangle_edge_ids.clear();
        }

        node_edge_offsets.push_back(node_edges.size());
    }

    std::vector<bool> is_boundary_edge;
    make_boundary_edges(triangulation_edges, edge_faces, is_boundary_edge);

    std::vector<bool> is_boundary_node;
    make_boundary_nodes(triangulation_edges, edge_faces, is_boundary_node);

    faces.clear();

    return polyhedron<BaseGraph, MaxNodesPerFace>(std::move(face_edges),
                                                  std::move(edge_faces),
                                                  std::move(is_boundary_node),
                                                  std::move(is_boundary_edge),
                                                  std::move(node_edges),
                                                  std::move(node_edge_offsets));
}

template<Topology BaseGraph, std::size_t MaxNodesPerFace>
polyhedron<BaseGraph, MaxNodesPerFace>::polyhedron(
    std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>>&&adjacent_edges,
    std::vector<std::array<face_id_type, FACE_COUNT_PER_EDGE>>&&adjacent_faces,
    std::vector<bool>&& is_boundary_node,
    std::vector<bool>&& is_boundary_edge,
    std::vector<edge_id_type>&&node_edges,
    std::vector<int>&&node_face_offsets)
    : _face_info(std::move(adjacent_edges)),
      _node_edges_offsets(std::move(node_face_offsets)),
      _node_edges(std::move(node_edges)),
      _edge_info(std::move(adjacent_faces)),
      _edge_links(),
      _is_boundary_node(std::move(is_boundary_node)),
      _is_boundary_edge(std::move(is_boundary_edge)),
      _boundary_edge_count(0),
      _boundary_node_count(0) {

    for (auto&& boundary : _is_boundary_edge) {
        _boundary_edge_count += boundary;
    }

    for (auto&& boundary : _is_boundary_node) {
        _boundary_node_count += boundary;
    }


    // make edge links
    _edge_links.reserve(_edge_info.size());
    edge_id_type current = 0;
    for (auto&&edge_info: _edge_info) {
        int count = 0;
        _edge_links.emplace_back();
        for (auto&&face: edge_info) {
            if (is_none(face)) continue;
            for (auto&&other_edge: _face_info[face]) {
                if (other_edge != current)
                    _edge_links.back()[count++] = other_edge;
            }
        }

        while (count < _edge_links.back().size()) {
            _edge_links.back()[count++] = none_value<edge_id_type>;
        }

        ++current;
    }

    // // check integrity
    // std::size_t num_edges = 3 * face_count(); // 3 edges per face
    // std::size_t non_boundary_edges = edge_count() - boundary_edge_count();

    // // non boundary edges have been counted twice
    // num_edges -= non_boundary_edges;

    // assert(num_edges == edge_count());
}

