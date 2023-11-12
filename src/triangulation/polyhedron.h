#pragma once

#include "../graph/adjacency_list.h"

template<typename T>
concept EdgeLinks = requires {
    typename T::edge_id_type;
} && requires(T t, typename T::edge_id_type edge) {
    { t.edges(edge) } -> std::convertible_to<typename T::edge_id_type>;
    { t.inverse_edge(edge) } -> std::convertible_to<typename T::edge_id_type>;
};


/**
 * stores the topology of a polyhedron. Provides O(1) access to adjacent edges for given nodes/edges
 */
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
class polyhedron {
public:
    using node_id_type = typename BaseGraph::node_id_type;
    using edge_id_type = typename BaseGraph::edge_id_type;

    // forward and backward edges for 2 faces minus the local edge itself
    // plus the inverse to the local edge
    // static constexpr std::size_t EDGE_COUNT = (MaxNodesPerFace - 1) * 2 * 2 + 1;

    // only store inverse edge and two adjacent edges
    static constexpr std::size_t EDGE_COUNT = 5;


    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::array<edge_id_type, EDGE_COUNT>);

private:
    static_assert(sizeof(std::array<edge_id_type, EDGE_COUNT>) == EDGE_COUNT * sizeof(edge_id_type));

    std::vector<std::array<edge_id_type, EDGE_COUNT>> _M_adjacent_edges;

    polyhedron(
            std::vector<std::array<edge_id_t, EDGE_COUNT>> &&__adjacent_edges)
            : _M_adjacent_edges(std::move(__adjacent_edges)) {};
public:

    /**
     * makes a polyhedron object from the given base graph
     * @param __base
     * @return
     */
    static polyhedron<BaseGraph, MaxNodesPerFace> make_polyhedron(const BaseGraph &__base,
                                                                  const std::vector<std::array<typename BaseGraph::node_id_type, MaxNodesPerFace>> &faces);

    /**
     * gets edges that belong to the faces bordering this edge
     * does not include the inverse to the given edge
     * @param __edge
     * @return
     */
    std::array<const edge_id_type, 4 * (MaxNodesPerFace - 1)> edges(edge_id_type __edge) const {
        auto inv = _M_adjacent_edges[__edge][0];
        std::array<const edge_id_type, 4 * (MaxNodesPerFace - 1)> result{
                _M_adjacent_edges[__edge][1],
                _M_adjacent_edges[__edge][2],
                _M_adjacent_edges[__edge][3],
                _M_adjacent_edges[__edge][4],
                _M_adjacent_edges[inv][1],
                _M_adjacent_edges[inv][2],
                _M_adjacent_edges[inv][3],
                _M_adjacent_edges[inv][4],
        };
        return result;
        //return std::span(_M_adjacent_edges[__edge]).template subspan<1, EDGE_COUNT - 1>();
    };

    /**
     * gets the id of the inverse edge for the given one
     * @param __edge
     * @return
     */
    edge_id_type inverse_edge(edge_id_type __edge) const {
        return _M_adjacent_edges[__edge][0];
    };


    /**
     * gets edges that belong to the faces bordering this node
     * @param __node
     * @return
     */
    // TODO get access to offset array
    std::span<const edge_id_type, std::dynamic_extent> node_edges(node_id_type __node) const {
        //return std::span(_M_adjacent_edges.edge(__node), _M_adjacent_edges.edge(__node + 1));
    };
};

