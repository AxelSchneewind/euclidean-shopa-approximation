#pragma once

#include "polyhedron.h"

#include "../routing/dijkstra_concepts.h"
#include "../util/list_intersection.h"
#include "../util/list_invert.h"

#include <array>
#include <vector>

namespace {
    template<std::size_t MaxNodesPerFace, Topology BaseGraph, TupleRange<MaxNodesPerFace> Faces, TupleRange<MaxNodesPerFace> FaceEdges, TupleRange<2> EdgeFaces>
    void make_face_edges(BaseGraph const &base,
                         Faces const &faces,
                         FaceEdges &face_edges,
                         EdgeFaces &edge_faces) {
        assert(std::ranges::size(faces) == std::ranges::size(face_edges));
        assert(std::ranges::size(edge_faces) == base.edge_count());

#pragma omp parallel
        {   // add edges to each triangle
            std::vector<typename BaseGraph::node_id_type> adjacent_edges;
#pragma omp for
            for (size_t face_index = 0; face_index < faces.size(); face_index++) {
                auto face = faces[face_index];

                adjacent_edges.clear();

                for (size_t i = 0; i < MaxNodesPerFace; ++i) {
                    auto node_id = face[i % MaxNodesPerFace];
                    auto node_id_n = face[(i + 1) % MaxNodesPerFace];

                    assert(node_id != node_id_n);
                    if (node_id > node_id_n)
                        std::swap(node_id, node_id_n);

                    auto edge_id = base.edge_id(node_id, node_id_n);
                    assert(!optional::is_none(edge_id));
                    adjacent_edges.emplace_back(edge_id);
                }

                std::ranges::sort(adjacent_edges);
                remove_duplicates_sorted(adjacent_edges);
                assert(adjacent_edges.size() == MaxNodesPerFace);

                // add edges array to face
                size_t index = 0;
                std::ranges::copy(adjacent_edges, face_edges[face_index].begin());
                assert(index <= MaxNodesPerFace);
            }
        }

        {   // add triangle to edges
            std::vector<char> triangle_count(base.edge_count(), 0);
            size_t face_index = 0;
            for (size_t face_index = 0; face_index < faces.size(); face_index++) {
                for (auto &&e: face_edges[face_index]) {
                    edge_faces[e][triangle_count[e]++] = face_index;
                }
            }

            // fill with -1 for edges that are only part of one face
#pragma omp parallel for
            for (size_t i = 0; i < base.edge_count(); ++i) {
                assert(triangle_count[i] >= 1);
                assert(triangle_count[i] <= 2);
                while (triangle_count[i] < 2)
                    edge_faces[i][triangle_count[i]++] = optional::none_value<typename BaseGraph::edge_id_type>;
            }
        }
    }

    template<Topology BaseGraph, std::ranges::random_access_range Out>
    void make_inverse_edges(const BaseGraph &base, Out &result) {
        assert(std::ranges::size(result) == base.edge_count());

#pragma omp parallel for
        for (size_t i = 0; i < base.edge_count(); ++i) {
            auto src = base.source(i);
            auto dest = base.destination(i);
            result[i] = base.edge_id(dest, src);
        }
    }

    template<std::size_t FacesPerEdge, Topology BaseTopology, TupleRange<FacesPerEdge> EdgeFaces, std::ranges::random_access_range Out>
    void make_boundary_nodes(BaseTopology const &triangulation,
                             EdgeFaces const &edge_faces,
                             Out &out) {
        assert(std::ranges::size(out) == triangulation.node_count());
        std::ranges::fill(out, false);

        // a vertex is a boundary vertex if there exists an edge that only has one adjacent face
#pragma omp parallel for
        for (size_t i = 0; i < triangulation.edge_count(); i++) {
            auto src = triangulation.source(i);
            auto dest = triangulation.destination(i);
            bool one_adjacent_face = optional::is_none(edge_faces[i][1]);
            out[src] = out[src] || one_adjacent_face;
            out[dest] = out[dest] || one_adjacent_face;
        }
    }

    template<std::size_t FacesPerEdge, Topology BaseTopology, TupleRange<FacesPerEdge> EdgeFaces, std::ranges::random_access_range Out>
    void make_boundary_edges(BaseTopology const &triangulation,
                             EdgeFaces const &edge_faces,
                             Out &out) {
        assert(std::ranges::size(out) == triangulation.edge_count());
        std::ranges::fill(out, false);

        // a vertex is a boundary vertex if there exists an edge that only has one adjacent face
#pragma omp parallel for
        for (size_t i = 0; i < triangulation.edge_count(); i++) {
            bool const one_adjacent_face = optional::is_none(edge_faces[i][1]);
            out[i] = one_adjacent_face;
        }
    }

    template<std::size_t FacesPerEdge, std::ranges::random_access_range Coords, Topology BaseTopology, TupleRange<FacesPerEdge> EdgeFaces, std::ranges::random_access_range OutBoundaryNodes, std::ranges::random_access_range OutBoundaryEdges, std::ranges::random_access_range OutConvexBoundaryNodes>
    void make_convex_boundary_nodes(Coords const &coordinates,
                                    BaseTopology const &triangulation,
                                    EdgeFaces const &edge_faces,
                                    OutBoundaryNodes &is_boundary_node,
                                    OutBoundaryEdges &is_boundary_edge,
                                    OutConvexBoundaryNodes &out) {
        assert(std::ranges::size(out) == triangulation.node_count());
        std::ranges::fill(out, false);

        // a vertex is a boundary vertex if there exists an edge that only has one adjacent face
        std::vector<long> nodes;
        for (size_t n = 0; n < triangulation.node_count(); n++) {
            if (!is_boundary_node[n]) continue;

            // get surrounding nodes
            nodes.clear();
            for (auto const &edge: triangulation.outgoing_edges(n)) {
                nodes.emplace_back(edge.destination);
            }
            for (auto const &edge: triangulation.incoming_edges(n)) {
                nodes.emplace_back(edge.destination);
            }

            // order nodes clockwise
            std::ranges::sort(nodes, std::less<long>{}, [&coordinates,n](auto node) { std::atan2(coordinates[node] - coordinates[n]); });

            // mark with false if there is an angle > 180º
            for (size_t index = 0; index < nodes.size(); index++) {
                if (angle(coordinates[index] - coordinates[n], coordinates[(index + 1 % nodes.size())] - coordinates[n]) >= std::numbers::pi / 2)
                    is_boundary_node[n] = false;
            }
        }
    }

    template<size_t FacesPerEdge, size_t EdgesPerFace, Topology BaseTopology, TupleRange<EdgesPerFace> FaceEdges, TupleRange<FacesPerEdge> EdgeFaces, std::ranges::range NodeEdges, std::ranges::range EdgeOffsets>
    void
    make_node_edges(BaseTopology const &triangulation_edges, FaceEdges const &face_edges, EdgeFaces const &edge_faces,
                    NodeEdges &node_edges, EdgeOffsets &node_edge_offsets) {
        auto node_edges_iterator{node_edges.begin()};
        auto node_edges_end{node_edges.end()};

        auto offsets_iterator{node_edge_offsets.begin()};
        auto offsets_end{node_edge_offsets.end()};

        std::vector<long> adjacent_edge_ids;
        std::vector<long> triangle_edge_ids;
        for (std::size_t node = 0; node < triangulation_edges.node_count(); ++node) {
            adjacent_edge_ids.clear();
            triangle_edge_ids.clear();

            // get edges and triangles adjacent to node
            for (auto &&edge: triangulation_edges.outgoing_edges(node)) {
                auto edge_id = triangulation_edges.edge_id(node, edge.destination);
                assert(!triangulation_edges.has_edge(edge.destination, node));
                adjacent_edge_ids.emplace_back(edge_id);

                for (auto const &triangle: edge_faces[edge_id]) {
                    if (optional::is_none(triangle)) continue;
                    for (auto &&other_edge: face_edges[triangle])
                        triangle_edge_ids.emplace_back(other_edge);
                }
            }
            for (auto &&edge: triangulation_edges.incoming_edges(node)) {
                auto edge_id = triangulation_edges.edge_id(edge.destination, node);
                assert(!triangulation_edges.has_edge(node, edge.destination));
                adjacent_edge_ids.emplace_back(edge_id);

                for (auto const &triangle: edge_faces[edge_id]) {
                    if (optional::is_none(triangle)) continue;
                    for (auto &&other_edge: face_edges[triangle])
                        triangle_edge_ids.emplace_back(other_edge);
                }
            }

            std::ranges::sort(triangle_edge_ids);
            std::ranges::sort(adjacent_edge_ids);

            // get edges that are only reachable via a triangle     TODO: use std::ranges algorithms
            remove_duplicates_sorted(triangle_edge_ids);
            remove_duplicates_sorted(adjacent_edge_ids);
            if (!triangle_edge_ids.empty()) {
                int k = set_minus_sorted<int>(triangle_edge_ids, adjacent_edge_ids, triangle_edge_ids);
                triangle_edge_ids.resize(k);
            }

            // node_edge_offsets.emplace_back(node_edges.size());
            *offsets_iterator = std::ranges::distance(node_edges.begin(), node_edges_iterator);
            ++offsets_iterator;
            for (auto &&e: triangle_edge_ids) {
                // node_edges.emplace_back(e);
                *node_edges_iterator = e;
                ++node_edges_iterator;
            }

            adjacent_edge_ids.clear();
            triangle_edge_ids.clear();
        }

        // node_edge_offsets.emplace_back(node_edges.size());
        *offsets_iterator = std::ranges::size(node_edges);
        ++offsets_iterator;
    }
}

// can be generalized to any type of polyhedron (using variable instead of statically sized arrays)
template<std::integral IndexType, std::size_t MaxNodesPerFace>
template<Topology BaseGraph, TupleRange<MaxNodesPerFace> FaceRange>
polyhedron<IndexType, MaxNodesPerFace>
polyhedron<IndexType, MaxNodesPerFace>::make_polyhedron(BaseGraph const &triangulation_edges,
                                                        FaceRange const &faces) {
    constexpr std::size_t face_count_per_edge = polyhedron<IndexType, MaxNodesPerFace>::FACE_COUNT_PER_EDGE;
    constexpr std::size_t edge_count_per_face = polyhedron<IndexType, MaxNodesPerFace>::EDGE_COUNT_PER_FACE;
    constexpr std::size_t node_count_per_face = polyhedron<IndexType, MaxNodesPerFace>::EDGE_COUNT_PER_FACE;

    // get all adjacent edges for each face
    // and all adjacent edge_faces for each edge
    std::vector<std::array<edge_id_type, edge_count_per_face>> face_edges(faces.size());
    std::vector<std::array<face_id_type, face_count_per_edge>> edge_faces(triangulation_edges.edge_count());
    make_face_edges<edge_count_per_face>(triangulation_edges, faces, face_edges, edge_faces);

    // make edges reachable from base nodes
    std::vector<edge_id_type> node_edges(2 * triangulation_edges.edge_count());
    std::vector<int> node_edge_offsets(triangulation_edges.node_count() + 1);
    make_node_edges<face_count_per_edge, edge_count_per_face>(triangulation_edges, face_edges, edge_faces, node_edges,
                                                              node_edge_offsets);

    std::vector<bool> is_boundary_edge(triangulation_edges.edge_count());
    make_boundary_edges<face_count_per_edge>(triangulation_edges, edge_faces, is_boundary_edge);

    std::vector<bool> is_boundary_node(triangulation_edges.node_count());
    make_boundary_nodes<face_count_per_edge>(triangulation_edges, edge_faces, is_boundary_node);

    return {std::move(face_edges),
            std::move(edge_faces),
            std::move(is_boundary_node),
            std::move(is_boundary_edge),
            std::move(node_edges),
            std::move(node_edge_offsets)};
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
polyhedron<IndexType, MaxNodesPerFace>::polyhedron(
        std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>> &&adjacent_edges,
        std::vector<std::array<face_id_type, FACE_COUNT_PER_EDGE>> &&adjacent_faces,
        std::vector<bool> &&is_boundary_node, std::vector<bool> &&is_boundary_edge,
        std::vector<edge_id_type> &&node_edges, std::vector<int> &&node_edge_offsets)
        : _boundary_node_count{std::ranges::count(is_boundary_node, true)}
        , _boundary_edge_count{std::ranges::count(is_boundary_edge, true)}
        , _is_boundary_node{std::move(is_boundary_node)}
        , _is_boundary_edge{std::move(is_boundary_edge)}
        , _face_info{std::move(adjacent_edges)}
        , _edge_info{std::move(adjacent_faces)}
        , _edge_links{}
        , _node_edges_offsets{std::move(node_edge_offsets)}
        , _node_edges{std::move(node_edges)} {

    // make edge links
    _edge_links.reserve(_edge_info.size());
    edge_id_type current = 0;
    for (auto &&edge_info: _edge_info) {
        size_t count = 0;

        // store reachable edges
        _edge_links.emplace_back();
        for (auto &&face: edge_info) {
            if (optional::is_none(face)) continue;
            for (auto &&other_edge: _face_info[face]) {
                if (other_edge != current)
                    _edge_links.back()[count++] = other_edge;
            }
        }
        std::ranges::sort(_edge_links.back().begin(), _edge_links.back().begin() + count);

        while (count < _edge_links.back().size()) {
            _edge_links.back()[count++] = optional::none_value<edge_id_type>;
        }

        ++current;
    }

    // sanity check: 3 * |faces| - |non_boundary_edges| == |edges|
    std::size_t num_edges = 3 * face_count(); // 3 edges per face
    std::size_t non_boundary_edges = edge_count() - boundary_edge_count();

    // non boundary edges have been counted twice (once for each adjacent face)
    num_edges -= non_boundary_edges;

    assert(num_edges == edge_count());
}


template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::span<const typename polyhedron<IndexType, MaxNodesPerFace>::edge_id_type, std::dynamic_extent>
polyhedron<IndexType, MaxNodesPerFace>::edges(edge_id_type edge) const {
    assert(!optional::is_none(edge));
    if (_is_boundary_edge[edge]) [[unlikely]] {
        assert(optional::is_none(_edge_links[edge][EDGE_COUNT_PER_FACE - 1]));
        return {_edge_links[edge].begin(), _edge_links[edge].begin() + (EDGE_COUNT_PER_FACE - 1)};
    } else {
        return {_edge_links[edge].begin(), _edge_links[edge].end()};
    }
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::span<const typename polyhedron<IndexType, MaxNodesPerFace>::edge_id_type, std::dynamic_extent>
polyhedron<IndexType, MaxNodesPerFace>::node_edges(node_id_type node) const {
    assert(node >= 0 && static_cast<size_t>(node) < _node_edges_offsets.size() - 1);
    return {
            _node_edges.begin() + _node_edges_offsets[node],
            _node_edges.begin() + _node_edges_offsets[node + 1]
    };
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::span<const typename polyhedron<IndexType, MaxNodesPerFace>::edge_id_type, polyhedron<IndexType, MaxNodesPerFace>::EDGE_COUNT_PER_FACE>
polyhedron<IndexType, MaxNodesPerFace>::face_edges(polyhedron::face_id_type face) const {
    assert(face >= 0 && static_cast<size_t>(face) < _face_info.size());
    return {_face_info[face]};
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::span<const typename polyhedron<IndexType, MaxNodesPerFace>::face_id_type, polyhedron<IndexType, MaxNodesPerFace>::FACE_COUNT_PER_EDGE>
polyhedron<IndexType, MaxNodesPerFace>::edge_faces(edge_id_type edge) const {
    assert(edge >= 0 && static_cast<size_t>(edge) < _edge_info.size());
    return {_edge_info[edge]};
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
bool polyhedron<IndexType, MaxNodesPerFace>::is_boundary_node(node_id_type node) const {
    assert(node >= 0 && static_cast<size_t>(node) < _is_boundary_node.size());
    return _is_boundary_node[node];
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
bool polyhedron<IndexType, MaxNodesPerFace>::is_boundary_edge(edge_id_type edge) const {
    assert(edge >= 0 && static_cast<size_t>(edge) < _is_boundary_edge.size());
    return _is_boundary_edge[edge];
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::size_t polyhedron<IndexType, MaxNodesPerFace>::boundary_edge_count() const {
    return _boundary_node_count;
}

template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::size_t polyhedron<IndexType, MaxNodesPerFace>::face_count() const { return _face_info.size(); }

template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::size_t polyhedron<IndexType, MaxNodesPerFace>::edge_count() const { return _edge_info.size(); }

template<std::integral IndexType, std::size_t MaxNodesPerFace>
std::size_t polyhedron<IndexType, MaxNodesPerFace>::node_count() const { return _node_edges_offsets.size() - 1; }
