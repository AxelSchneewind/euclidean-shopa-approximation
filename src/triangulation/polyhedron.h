#pragma once

#include "../graph/base_types.h"
#include "../routing/dijkstra_concepts.h"

#include "../util/set_minus.h"
#include "../util/remove_duplicates.h"

#include <array>
#include <cstddef>
#include <vector>

/**
 * stores the topology of a polyhedron. Provides O(1) access to adjacent edges for given nodes/edges
 */
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
class polyhedron {
public:
    using node_id_type = typename BaseGraph::node_id_type;
    using edge_id_type = typename BaseGraph::edge_id_type;
    using face_id_type = int;

    // edges that are part of a face
    static constexpr std::size_t EDGE_COUNT_PER_FACE = MaxNodesPerFace;
    static constexpr std::size_t FACE_COUNT_PER_EDGE = 2;

private:
    static_assert(sizeof(std::array<edge_id_type, EDGE_COUNT_PER_FACE>) == EDGE_COUNT_PER_FACE * sizeof(edge_id_type));

    using edge_info_type = std::array<face_id_type, FACE_COUNT_PER_EDGE>;
    using edge_link_type = std::array<edge_id_type, (EDGE_COUNT_PER_FACE - 1) * FACE_COUNT_PER_EDGE>;

    std::size_t _boundary_node_count{0};
    std::size_t _boundary_edge_count{0};

    // TODO add flags for convexity
    std::vector<bool> _is_boundary_node;
    std::vector<bool> _is_boundary_edge;

    // for each triangle
    std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>> _face_info;

    // for each edge
    std::vector<edge_info_type> _edge_info;
    std::vector<edge_link_type> _edge_links;

    // for each node, store edges that are reachable by crossing a face
    std::vector<int> _node_edges_offsets;
    std::vector<edge_id_type> _node_edges;


    polyhedron(std::vector<std::array<edge_id_type, EDGE_COUNT_PER_FACE>>&& adjacent_edges,
               std::vector<std::array<face_id_type, FACE_COUNT_PER_EDGE>>&& adjacent_faces,
               std::vector<bool>&& is_boundary_node,
               std::vector<bool>&& is_boundary_edge,
               std::vector<edge_id_type>&& node_edges,
               std::vector<int>&& node_edge_offsets);

public:
    static constexpr std::size_t SIZE_PER_NODE = 0;
    static constexpr std::size_t SIZE_PER_EDGE = sizeof(edge_info_type);

    std::size_t node_count() const;

    std::size_t edge_count() const;

    std::size_t face_count() const;

    std::size_t boundary_edge_count() const;

    bool is_boundary_edge(node_id_type node) const;

    bool is_boundary_node(node_id_type node) const;

    /**
     * @return
     */
    [[gnu::cold]]
    static polyhedron make_polyhedron(BaseGraph const&triangulation_edges,
                                    std::vector<std::array<node_id_type, MaxNodesPerFace>> &&faces);

    [[gnu::hot]]
    std::span<const face_id_type, FACE_COUNT_PER_EDGE> edge_faces(edge_id_type edge) const;

    [[gnu::hot]]
    std::span<const edge_id_type, EDGE_COUNT_PER_FACE> face_edges(face_id_type face) const;

    [[gnu::hot]]
    std::span<const edge_id_type, std::dynamic_extent> node_edges(node_id_type node) const;

    /**
     * gets edges that belong to the faces bordering this edge
     * @param edge
     * @return
     */
    [[gnu::hot]]
    std::span<const edge_id_type, std::dynamic_extent> edges(edge_id_type edge) const;
};
