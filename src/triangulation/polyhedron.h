#pragma once

#include "../graph/adjacency_list.h"

/**
 * stores the topology of a polyhedron. Provides O(1) access to adjacent faces/edges for given nodes/edges
 */
template<Topology BaseGraph, std::size_t MaxNodesPerFace>
class polyhedron {
public:
    using adjacency_list_type = adjacency_list<node_id_t, std::nullptr_t>;

    using node_id_type = typename BaseGraph::node_id_type;
    using edge_id_type = typename BaseGraph::edge_id_type;

    static constexpr std::size_t edge_count = (MaxNodesPerFace - 1) * 2;

private:
    BaseGraph _M_base_graph;
    adjacency_list<node_id_type, std::array<edge_id_type, edge_count>> _M_adjacent_edges;

    polyhedron(const BaseGraph &__base_graph,
                  adjacency_list<node_id_type, std::array<node_id_type, edge_count>> __third_points)
            : _M_base_graph(__base_graph), _M_adjacent_edges(__third_points) {};
public:

    /**
     * makes a polyhedron objcet from the given base graph
     * @param __base
     * @return
     */
    static polyhedron make_polyhedron(const BaseGraph& __base);

    /*
     * gets edges that belong to the faces bordering this edge
     */
    std::array<edge_id_type, edge_count> edges(const edge_id_type &__edge) const {
        return _M_adjacent_edges.edge(__edge);
        //return std::span<const edge_id_type , edge_count>(_M_adjacent_edges.edge(__edge));
    };

};

