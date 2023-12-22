#pragma once

#include "base_types.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

#include "../routing/dijkstra_concepts.h"
#include "../util/counting_iterator.h"

template<typename NodeId, typename E = void>
struct internal_adjacency_list_edge {
    NodeId destination;
    E info;
};


template<typename NodeId, typename E = void>
struct adjacency_list_edge {
    NodeId source;
    NodeId destination;
    E info;

    bool operator==(const adjacency_list_edge &__other) const = default;

    bool operator!=(const adjacency_list_edge &__other) const = default;

    operator internal_adjacency_list_edge<NodeId, E>() const { return {destination, info}; }
};

template<typename NodeId>
struct internal_adjacency_list_edge<NodeId, void> {
    NodeId destination;

    internal_adjacency_list_edge() = default;

    internal_adjacency_list_edge(NodeId destination) : destination(destination) {}
};

template<typename NodeId>
struct adjacency_list_edge<NodeId, void> {
    NodeId source;
    NodeId destination;
    static constexpr std::nullptr_t info = nullptr;

    adjacency_list_edge() = default;

    adjacency_list_edge(NodeId source, NodeId destination, std::nullptr_t info) : source(source),
                                                                                  destination(destination) {}

    bool operator==(const adjacency_list_edge &__other) const = default;

    bool operator!=(const adjacency_list_edge &__other) const = default;

    operator internal_adjacency_list_edge<NodeId>() const { return {destination}; }
};

/**
 * stores a directed graph. Provides O(1) access to outgoing edges for any node
 * @tparam NodeId the type by which nodes are identified
 * @tparam E information stored on each edge
 */
template<typename NodeId, typename E>
class unidirectional_adjacency_list {
public:
    using edge_index_type = edge_id_t;
    using node_id_type = NodeId;
    using edge_info_type = E;

    class adjacency_list_builder {
    private:
        size_t _M_node_count;
        size_t _M_edge_count;

        std::vector<adjacency_list_edge<NodeId, E>> _M_edges;

    public:
        using edge_type = adjacency_list_edge<NodeId, E>;

        adjacency_list_builder() : _M_node_count(0), _M_edge_count(0) {};

        adjacency_list_builder(adjacency_list_builder &&__other) = default;

        adjacency_list_builder(const adjacency_list_builder &__other) = default;

        adjacency_list_builder(size_t __node_count) : _M_node_count(__node_count), _M_edge_count(0) {
            _M_edges.reserve(_M_node_count);
        };

        ~adjacency_list_builder() = default;

        adjacency_list_builder &operator=(adjacency_list_builder &&__other) = default;

        adjacency_list_builder &operator=(const adjacency_list_builder &__other) = default;

        edge_type& edge(std::size_t index) { return _M_edges[index]; }

        void sort_edges();
        void remove_duplicates();

        std::span<edge_type, std::dynamic_extent> edges() { return {_M_edges.begin(), _M_edges.end()};}

        void add_edges_from_triangulation(std::vector<std::array<node_id_type, 3>> const& faces);
        void add_edges_from_triangulation(std::vector<std::array<node_id_type, 3>> && faces);

        void add_edges(std::vector<edge_type> const& edges);
        void add_edges(std::vector<edge_type> && edges);

        void add_node(NodeId __node);

        void add_edge(adjacency_list_edge<NodeId, E> const& __edge) { _M_edges.emplace_back(__edge); };

        void add_edge(NodeId __source, NodeId __destination, E __info);
        void add_edge(NodeId __source, NodeId __destination);

        void insert_backward_edges();

        unidirectional_adjacency_list<NodeId, E> get();
    };

    static constexpr size_t SIZE_PER_NODE = sizeof(edge_index_type);
    static constexpr size_t SIZE_PER_EDGE = sizeof(NodeId) + sizeof(internal_adjacency_list_edge<NodeId, E>);

private:
    size_t _M_node_count;
    size_t _M_edge_count;

    // per node
    std::vector<edge_index_type> _M_offsets;

    // per edge
    std::vector<NodeId> _M_sources;
    std::vector<internal_adjacency_list_edge<NodeId, E> > _M_edges;

    inline edge_index_type offset(NodeId __node) const;

    inline edge_index_type offset_next(NodeId __node) const;


public:
    ~unidirectional_adjacency_list();

    unidirectional_adjacency_list(size_t __node_count,
                                  std::vector<adjacency_list_edge<NodeId, E> > &&__edges);

    unidirectional_adjacency_list(std::vector<edge_id_t> &&__offsets,
                                  std::vector<NodeId> &&__sources,
                                  std::vector<internal_adjacency_list_edge<NodeId, E>> &&__edges);

    // move constructor
    unidirectional_adjacency_list(unidirectional_adjacency_list &&__other) noexcept;

    unidirectional_adjacency_list<NodeId, E> &operator=(unidirectional_adjacency_list<NodeId, E> &&__other) = default;

    // copy constructor
    unidirectional_adjacency_list(const unidirectional_adjacency_list &__other) = delete;

    unidirectional_adjacency_list<NodeId, E> &
    operator=(const unidirectional_adjacency_list<NodeId, E> &__other) = delete;

    unidirectional_adjacency_list<NodeId, E> inverse() const;

    inline bool contains_node(node_id_type __node_id) const;

    inline bool contains_edge(edge_index_type __edge_index) const;


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
     * enumerates all edge ids
     * @return
     */
    counter<edge_index_type> edge_ids() const {
        return {edge_count()};
    }

    /**
     * enumerates all node ids
     * @return
     */
    counter<edge_index_type> node_ids() const {
        return {node_count()};
    }

    /**
     * get the id of the source node for the edge with given index
     * @param __edge
     * @return
     */
    inline NodeId source(edge_index_type __edge) const;

    /**
     * get the id of the destination for the edge with given index
     * @param __edge
     * @return
     */
    inline NodeId destination(edge_index_type __edge) const;

    /**
     * get the edge information at the given index
     * @param __edge
     * @return
     */
    inline E edge(edge_index_type __edge) const;

    /**
     * get the index of (source, destination)
     * @param __source
     * @param __dest
     * @return the index, or NO_EDGE_ID otherwise
     */
    inline edge_index_type edge_id(NodeId __source, NodeId __dest) const;

    /**
     * get the index of any edge (source, w) in graph
     * @param __source
     * @return
     */
    inline edge_index_type edge_id(NodeId __source) const;

    /**
     * check if (source, destination) in graph
     * @param __source
     * @param __dest
     * @return
     */
    inline bool has_edge(NodeId __source, NodeId __dest) const;

    /**
     * gets the distance info pairs for outgoing edges from the given node
     * @param __node the source node
     * @return a span over the destination/cost pairs
     */
    inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
    outgoing_edges(NodeId __node) const;

    bool operator==(const unidirectional_adjacency_list<NodeId, E> &__other);
};


template<typename NodeId, typename E>
class std::hash<adjacency_list_edge<NodeId, E>>;