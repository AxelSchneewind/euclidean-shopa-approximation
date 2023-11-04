#pragma once

#include "base_types.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

#include "../routing/dijkstra_concepts.h"

template<typename NodeId, typename E>
struct internal_adjacency_list_edge {
    NodeId destination;
    E info;
};


template<typename NodeId, typename E>
struct adjacency_list_edge {
    NodeId source;
    NodeId destination;
    E info;

    bool operator==(const adjacency_list_edge &__other) const = default;
};


/**
 * stores a directed graph. Provides O(1) access to outgoing edges for any node
 * @tparam NodeId the type by which nodes are identified
 * @tparam E information stored on each edge
 */
template<typename NodeId, typename E>
class unidirectional_adjacency_list {
public:
    class adjacency_list_builder {
    private:
        size_t _M_node_count;
        size_t _M_edge_count;

        std::vector<edge_id_t> _M_offsets;
        std::vector<adjacency_list_edge<NodeId, E>> _M_edges;

    public:
        adjacency_list_builder() : _M_node_count(0), _M_edge_count(0) {};
        adjacency_list_builder(adjacency_list_builder&& __other) = default;
        adjacency_list_builder(const adjacency_list_builder& __other) = default;
        adjacency_list_builder(size_t __node_count) : _M_node_count(__node_count), _M_edge_count(0) { _M_edges.reserve(_M_node_count);};
        ~adjacency_list_builder() = default;

        adjacency_list_builder& operator=(adjacency_list_builder&& __other) = default;
        adjacency_list_builder& operator=(const adjacency_list_builder& __other) = default;

        void add_node(const NodeId &__node);

        void add_edge(const adjacency_list_edge<NodeId, E> &__edge) { _M_edges.push_back(__edge); };

        void add_edge(const NodeId &__source, const NodeId &__destination, const E &__info);

        unidirectional_adjacency_list<NodeId, E> get();
    };


    using edge_index_type = edge_id_t;
    using node_id_type = NodeId;
    using edge_info_type = E;

private:
    size_t _M_node_count;
    size_t _M_edge_count;
    std::vector<edge_id_t> _M_offsets;
    std::vector<NodeId> _M_sources;
    std::vector<internal_adjacency_list_edge<NodeId, E> > _M_edges;

    inline const edge_index_type &offset(const NodeId &__node) const;

    inline const edge_index_type &offset_next(const NodeId &__node) const;

public:
    ~unidirectional_adjacency_list();

    unidirectional_adjacency_list(size_t __node_count, size_t __edge_count, std::vector<edge_id_t> &&__offsets,
                                  std::vector<adjacency_list_edge<NodeId, E> > &&__edges);

    unidirectional_adjacency_list(size_t __node_count, size_t __edge_count, std::vector<edge_id_t> &&__offsets,
                                  std::vector<NodeId> &&__sources,
                                  std::vector<internal_adjacency_list_edge<NodeId, E>> &&__edges);

    // move constructor
    unidirectional_adjacency_list(unidirectional_adjacency_list &&__other) noexcept;
    unidirectional_adjacency_list<NodeId, E>& operator=(unidirectional_adjacency_list<NodeId,E>&& __other) = default;

    // copy constructor
    unidirectional_adjacency_list(const unidirectional_adjacency_list& __other) = default;
    unidirectional_adjacency_list<NodeId, E>& operator=(const unidirectional_adjacency_list<NodeId,E>& __other) = default;

    unidirectional_adjacency_list<NodeId, E> inverse() const;

    /**
     * get the number of nodes of this adjacency list
     * @return
     */
    inline size_t node_count() const;

    /**
     * get the number of edges stored in this adjacency list
     * @return
     */
    inline size_t edge_count() const;

    /**
     * get the id of the source node for the edge with given index
     * @param __edge
     * @return
     */
    inline const NodeId &source(const edge_index_type &__edge) const;

    /**
     * get the id of the destination for the edge with given index
     * @param __edge
     * @return
     */
    inline NodeId &destination(const edge_index_type &__edge);
    inline const NodeId &destination(const edge_index_type &__edge) const;

    /**
     * get the edge information at the given index
     * @param __edge
     * @return
     */
    inline E &edge(const edge_index_type &__edge) ;
    inline const E &edge(const edge_index_type &__edge) const;

    /**
     * get the index of the edge with given source and destination ids
     * @param __source
     * @param __dest
     * @return the index, or NO_EDGE_ID otherwise
     */
    inline edge_index_type edge_index(const NodeId &__source, const NodeId &__dest) const;

    /**
     * check if an edge from the source to destination node exists
     * @param __source
     * @param __dest
     * @return
     */
    inline bool has_edge(const NodeId &__source, const NodeId &__dest) const;

    /**
     * gets the distance info pairs for outgoing edges from the given node
     * @param __node
     * @return
     */
    inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent> outgoing_edges(const NodeId &__node) const;

    bool operator==(const unidirectional_adjacency_list<NodeId, E> &__other);
};

template<typename NodeId, typename E>
class std::hash<adjacency_list_edge<NodeId, E>>;