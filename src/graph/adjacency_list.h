#pragma once

#include "unidirectional_adjacency_list.h"
#include "../routing/dijkstra_concepts.h"
#include "../util/counting_iterator.h"
#include <memory>

template<typename E>
class adjacency_list {
public:
    using node_id_type = node_id_t;
    using edge_id_type = edge_id_t;
    using node_info_type = node_t;
    using distance_type = distance_t;
    using edge_info_type = E;
    using adjacency_list_type = adjacency_list<edge_info_type>;

private:

private:
    std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E>> m_forward;
    std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E>> m_backward;


    adjacency_list(std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E>> __forward,
                   std::shared_ptr<unidirectional_adjacency_list<node_id_type, E>> __backward);

    adjacency_list(const std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E>> &__forward,
                   const std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E>> &__backward)
            : m_forward(__forward), m_backward(__backward) {};

public:
    static adjacency_list<E>
    make_bidirectional(const std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E>> &__forward);

    static adjacency_list<E> make_bidirectional(unidirectional_adjacency_list<node_id_type, E> &&__forward);

    static adjacency_list<E>
    make_bidirectional_undirected(const std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E>> &__edges);

    static adjacency_list<E> make_bidirectional_undirected(unidirectional_adjacency_list<node_id_type, E> &&__forward);

    adjacency_list(const adjacency_list<E> &__other) noexcept
            : m_forward(__other.m_forward), m_backward(__other.m_backward) {}

    adjacency_list(adjacency_list<E> &&__other) noexcept
            : m_forward(std::move(__other.m_forward)), m_backward(std::move(__other.m_backward)) {}

    adjacency_list<E> &operator=(const adjacency_list<E> &other) = default;

    adjacency_list<E> &operator=(adjacency_list<E> &&other) = default;

    adjacency_list inverse() const;

    inline size_t node_count() const;

    inline size_t edge_count() const;

    std::span<const internal_adjacency_list_edge<node_id_type, edge_info_type>, std::dynamic_extent>
    outgoing_edges(const node_id_type &source) const { return m_forward->outgoing_edges(source); }

    std::span<const internal_adjacency_list_edge<node_id_type, edge_info_type>, std::dynamic_extent>
    incoming_edges(const node_id_type &destination) const { return m_backward->outgoing_edges(destination); }

    counter<node_id_type> node_ids() const { return counter((node_id_type)node_count()); };

    node_id_type source(const edge_id_type &id) const { return m_forward->source(id); };

    node_id_type destination(const edge_id_type &id) const { return m_forward->destination(id); };

    edge_info_type edge(const edge_id_type &id) const { return m_forward->edge(id); };

    edge_id_type edge_id(const node_id_type &source, const node_id_type &destination) const {
        return m_forward->edge_index(source, destination);
    }

    bool has_edge(const node_id_type &source, const node_id_type &destination) const { return m_forward->edge_index(source, destination) != NO_EDGE_ID; }

    bool operator==(const adjacency_list<E> &__other);
};


static_assert(Topology<adjacency_list<int>>);