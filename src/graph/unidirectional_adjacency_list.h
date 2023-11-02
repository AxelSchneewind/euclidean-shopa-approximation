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

    adjacency_list_edge();

    adjacency_list_edge(NodeId source, NodeId destination, E info);

    adjacency_list_edge<NodeId, E> invert() const;

    bool operator==(const adjacency_list_edge &other) const;
};


// class that stores edges (topology) of a m_adj_list
template<typename NodeId, typename E>
class unidirectional_adjacency_list {
public:
    unidirectional_adjacency_list(size_t node_count, size_t edge_count, std::vector<edge_id_t> &&offsets,
                                  std::vector<NodeId> &&sources,
                                  std::vector<internal_adjacency_list_edge<NodeId, E>> &&edges);

    class adjacency_list_builder {
    private:
        size_t m_node_count;
        size_t m_edge_count;

        std::vector<edge_id_t> offset;
        std::vector<adjacency_list_edge<NodeId, E>> edges;

    public:
        adjacency_list_builder() : m_node_count(0), m_edge_count(0) {};
        ~adjacency_list_builder() = default;
        adjacency_list_builder(size_t __node_count) : m_node_count(__node_count), m_edge_count(0) {};

        void add_node(const node_id_t &node);

        void add_edge(const adjacency_list_edge<NodeId, E> &edge) { edges.push_back(edge); };

        void add_edge(const node_id_t &source, const node_id_t &destination, const E &info);

        unidirectional_adjacency_list<NodeId, E> get();
    };

private:
    size_t nodecount;
    size_t edgecount;
    std::vector<edge_id_t> offsets;
    std::vector<NodeId> sources;
    std::vector<internal_adjacency_list_edge<NodeId, E> > edges;

public:
    ~unidirectional_adjacency_list();

    unidirectional_adjacency_list(size_t node_count, size_t edge_count);

    unidirectional_adjacency_list(size_t node_count, size_t edge_count, std::vector<edge_id_t> &&offsets,
                                  std::vector<adjacency_list_edge<NodeId, E> > &&edges);

    unidirectional_adjacency_list(size_t node_count, size_t edge_count, std::vector<edge_id_t> &&offsets,
                                  std::vector<internal_adjacency_list_edge<NodeId, E> > &&edges);

    // move constructor
    unidirectional_adjacency_list(unidirectional_adjacency_list &&other) noexcept;
    unidirectional_adjacency_list<NodeId, E>& operator=(unidirectional_adjacency_list<NodeId,E>&& __other) = default;

    unidirectional_adjacency_list<NodeId, E> inverse() const;

    inline size_t node_count() const;

    inline size_t edge_count() const;

    inline const edge_id_t &offset(const node_id_t &__node) const;

    inline const edge_id_t &offset_next(const node_id_t &__node) const;

    inline node_id_t &source(const edge_id_t &__edge) ;
    inline const node_id_t &source(const edge_id_t &__edge) const;

    inline node_id_t &destination(const edge_id_t &__edge) ;
    inline const node_id_t &destination(const edge_id_t &__edge) const;

    inline E &edge(const edge_id_t &__edge) ;
    inline const E &edge(const edge_id_t &__edge) const;

    inline adjacency_list_edge<NodeId, E> adjlist_edge(const edge_id_t &__edge) const;

    inline edge_id_t edge_index(const node_id_t &__source, const node_id_t &__dest) const;

    inline bool has_edge(const node_id_t &__source, const node_id_t &__dest) const;

    inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent> outgoing_edges(const node_id_t &__node) const;

    bool operator==(const unidirectional_adjacency_list<NodeId, E> &__other);
};

template<typename NodeId, typename E>
class std::hash<adjacency_list_edge<NodeId, E>>;