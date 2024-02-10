#pragma once

#include "../routing/dijkstra_concepts.h"
#include "../util/counting_iterator.h"
#include "unidirectional_adjacency_list.h"
#include <memory>

/**
 * stores a topology and provides O(1) access to incoming and outgoing edges for any node
 * @tparam E the information stored for each node
 */
template<typename NodeId, typename E = void>
class adjacency_list {
public:
    using node_id_type = NodeId;
    using edge_id_type = unidirectional_adjacency_list<NodeId, E>::edge_index_type;
    using edge_info_type = E;
    using type = adjacency_list<NodeId, E>;

    using builder = unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder;

    static constexpr size_t SIZE_PER_NODE = 2 * unidirectional_adjacency_list<NodeId, E>::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = 2 * unidirectional_adjacency_list<NodeId, E>::SIZE_PER_EDGE;

private:
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> _forward;
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> _backward;


    adjacency_list(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> forward,
                   std::shared_ptr<unidirectional_adjacency_list<NodeId, E>> backward);

    adjacency_list(const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &forward,
                   const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &backward);

public:
    // copy constructor/assignment
    adjacency_list(const adjacency_list<NodeId, E> &other) noexcept;

    adjacency_list<NodeId, E> &operator=(const adjacency_list<NodeId, E> &other) = default;

    // move constructor/assignment
    adjacency_list(adjacency_list<NodeId, E> &&other) noexcept;

    adjacency_list<NodeId, E> &operator=(adjacency_list<NodeId, E> &&other) = default;

    ~adjacency_list() = default;

    /**
     * number of nodes
     * @return
     */
    size_t node_count() const;

    /**
     * number of edges
     * @return
     */
    size_t edge_count() const;

    /**
     * returns a span with the destination/info pairs for the given source node
     * @param source
     * @param from the node from where destination is reached (can be filtered out)
     * @return
     */
    std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
    outgoing_edges(NodeId source, NodeId from) const;

    std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent> outgoing_edges(NodeId source) const;


    /**
     * returns a span with the source/info pairs for the given destination node
     * @param destination
     * @param from the node from where destination is reached (can be filtered out)
     * @return
     */
    std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
    incoming_edges(NodeId destination, NodeId from) const;

    std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
    incoming_edges(NodeId destination) const;

    /**
     * returns an iterator over all node ids
     * @return
     */
    counter<NodeId> node_ids() const;

    /**
     * returns an iterator over all edge ids
     * @return
     */
    counter<edge_id_type> edge_ids() const;

    NodeId source(edge_id_type id) const;

    NodeId destination(edge_id_type id) const;

    E edge(edge_id_type id) const;

    /**
     * get the id for the edge (source, destination)
     * @param source
     * @param destination
     * @return
     */
    edge_id_type edge_id(NodeId source, NodeId destination) const;

    /**
     * get the id for any edge (source, destination)
     * @param source
     * @return
     */
    edge_id_type edge_id(NodeId source) const;

    /**
     * checks if the edge (source, destination) exists
     * @param source
     * @param destination
     * @return
     */
    bool has_edge(NodeId source, NodeId destination) const;

    bool operator==(const adjacency_list<NodeId, E> &other);

    /**
     * makes a directed bidirectional adjacency list from the given uni-directional one, i.e. generates a uni-directional adjacency list for the backward edges
     * @param forward
     * @return
     */
    static adjacency_list<NodeId, E>
    make_bidirectional(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> forward);

    /**
     * makes a directed bidirectional adjacency list from the given uni-directional one, i.e. generates a uni-directional adjacency list for the backward edges
     * @param forward
     * @return
     */
    static adjacency_list<NodeId, E> make_bidirectional(unidirectional_adjacency_list<NodeId, E> &&forward);

    /**
     * makes an undirected bidirectional adjacency list from the given uni-directional one, i.e. uses it for backward and forward edges
     * @param edges
     * @return
     */
    static adjacency_list<NodeId, E>
    make_bidirectional_undirected(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> edges);

    /**
     * makes an undirected bidirectional adjacency list from the given uni-directional one, i.e. uses it for backward and forward edges
     * @param edges
     * @return
     */
    static adjacency_list<NodeId, E>
    make_bidirectional_undirected(unidirectional_adjacency_list<NodeId, E> &&forward);

    /**
     * inverts this adjacency list
     * @param other
     * @return
     */
    static adjacency_list<NodeId, E> invert(const adjacency_list<NodeId, E> &other);
};

static_assert(Topology<adjacency_list<int, int>>);
