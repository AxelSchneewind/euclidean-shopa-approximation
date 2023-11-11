#pragma once

#include "../routing/dijkstra_concepts.h"
#include "../util/counting_iterator.h"
#include "unidirectional_adjacency_list.h"
#include <memory>

/**
 * stores a topology and provides O(1) access to incoming and outgoing edges for any node
 * @tparam E the information stored for each node
 */
template<typename NodeId, typename E>
class adjacency_list {
public:
    using node_id_type = NodeId;
    using edge_id_type = unidirectional_adjacency_list<NodeId, E>::edge_index_type;
    using edge_info_type = E;
    using type = adjacency_list<NodeId, E>;

    using builder = unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder;

private:
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> _M_forward;
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> _M_backward;


    adjacency_list(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> __forward,
                   std::shared_ptr<unidirectional_adjacency_list<NodeId, E>> __backward);

    adjacency_list(const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &__forward,
                   const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &__backward);

public:
    // copy constructor/assignment
    adjacency_list(const adjacency_list<NodeId, E> &__other) noexcept;
    adjacency_list<NodeId, E> &operator=(const adjacency_list<NodeId, E> &__other) = default;

    // move constructor/assignment
    adjacency_list(adjacency_list<NodeId, E> &&__other) noexcept;
    adjacency_list<NodeId, E> &operator=(adjacency_list<NodeId, E> &&__other) = default;

    ~adjacency_list() = default;

    /**
     * number of nodes
     * @return
     */
    inline size_t node_count() const;

    /**
     * number of edges
     * @return
     */
    inline size_t edge_count() const;

    /**
     * returns a span with the destination/info pairs for the given source node
     * @param __source
     * @return
     */
    std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent> outgoing_edges(NodeId __source) const;

    /**
     * returns a span with the source/info pairs for the given destination node
     * @param __destination
     * @return
     */
    std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
    incoming_edges(NodeId __destination) const;

    /**
     * returns an iterator over all node ids
     * @return
     */
    counter<NodeId> node_ids() const;

    NodeId source(edge_id_type __id) const;

    NodeId destination(edge_id_type __id) const;

    E edge(edge_id_type __id) const;

    edge_id_type edge_id(NodeId __source, NodeId __destination) const;

    bool has_edge(NodeId __source, NodeId __destination) const;

    bool operator==(const adjacency_list<NodeId, E> &__other);

    static adjacency_list<NodeId, E>
    make_bidirectional(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> __forward);

    static adjacency_list<NodeId, E> make_bidirectional(unidirectional_adjacency_list<NodeId, E> &&__forward);

    static adjacency_list<NodeId, E>
    make_bidirectional_undirected(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> __edges);

    static adjacency_list<NodeId, E>
    make_bidirectional_undirected(unidirectional_adjacency_list<NodeId, E> &&__forward);

    static adjacency_list<NodeId, E> invert(const adjacency_list<NodeId, E> &__other);
};

static_assert(Topology<adjacency_list<int, int>>);
