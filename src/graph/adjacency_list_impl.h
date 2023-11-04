#pragma once

#include "adjacency_list.h"

#include <memory>


template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional_undirected(unidirectional_adjacency_list<NodeId, E> &&__forward) {
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > fwd(
            new unidirectional_adjacency_list<NodeId, E>(std::move(__forward)));
    return make_bidirectional_undirected(fwd);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional_undirected(
        const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > &__edges) {
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > forward(__edges);
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > backward(__edges);

    return adjacency_list<NodeId, E>(forward, backward);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional(unidirectional_adjacency_list<NodeId, E> &&__forward) {
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > fwd(
            new unidirectional_adjacency_list<NodeId, E>(std::move(__forward)));
    return make_bidirectional(fwd);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional(
        const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > &__forward) {
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > forward(__forward);
    std::shared_ptr<unidirectional_adjacency_list<NodeId, E> > backward(
            new unidirectional_adjacency_list<NodeId, E>(__forward->inverse()));

    return adjacency_list<NodeId, E>(forward, backward);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E> > __forward,
                                          std::shared_ptr<unidirectional_adjacency_list<NodeId, E> > __backward)
        : _M_forward(__forward), _M_backward(__backward) {}

template<typename NodeId, typename E>
bool
adjacency_list<NodeId, E>::operator==(const adjacency_list<NodeId, E> &__other) {
    return _M_forward == __other._M_forward;
}

template<typename NodeId, typename E>
size_t
adjacency_list<NodeId, E>::node_count() const {
    return _M_forward->node_count();
}

template<typename NodeId, typename E>
size_t
adjacency_list<NodeId, E>::edge_count() const {
    return _M_forward->edge_count();
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::invert(const adjacency_list<NodeId, E> &__other) {
    return adjacency_list<NodeId, E>(__other._M_backward, __other._M_forward);
}


template<typename NodeId, typename E>
bool adjacency_list<NodeId, E>::has_edge(const NodeId &__source, const NodeId &__destination) const {
    return _M_forward->edge_index(__source, __destination);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::edge_id_type
adjacency_list<NodeId, E>::edge_id(const NodeId &__source, const NodeId &__destination) const {
    return _M_forward->edge_index(__source, __destination);
}


template<typename NodeId, typename E>
const E& adjacency_list<NodeId, E>::edge(const adjacency_list::edge_id_type &__id) const { return _M_forward->edge(__id); }

template<typename NodeId, typename E>
const NodeId& adjacency_list<NodeId, E>::destination(const adjacency_list::edge_id_type &__id) const {
    return _M_forward->destination(__id);
}

template<typename NodeId, typename E>
const NodeId& adjacency_list<NodeId, E>::source(const adjacency_list::edge_id_type &__id) const {
    return _M_forward->source(__id);
}

template<typename NodeId, typename E>
counter<NodeId> adjacency_list<NodeId, E>::node_ids() const { return counter((NodeId) node_count()); }

template<typename NodeId, typename E>
std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
adjacency_list<NodeId, E>::incoming_edges(const NodeId &__destination) const {
    return _M_backward->outgoing_edges(__destination);
}

template<typename NodeId, typename E>
std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
adjacency_list<NodeId, E>::outgoing_edges(const NodeId &__source) const {
    return _M_forward->outgoing_edges(__source);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(adjacency_list<NodeId, E> &&__other) noexcept
        : _M_forward(std::move(__other._M_forward)), _M_backward(std::move(__other._M_backward)) {}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(const adjacency_list<NodeId, E> &__other) noexcept
        : _M_forward(__other._M_forward), _M_backward(__other._M_backward) {}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(
        const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &__forward,
        const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &__backward)
        : _M_forward(__forward), _M_backward(__backward) {}
