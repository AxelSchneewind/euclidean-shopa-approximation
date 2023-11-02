#pragma once

#include "adjacency_list.h"

#include <memory>


template<typename E>
adjacency_list<E>
adjacency_list<E>::make_bidirectional_undirected(unidirectional_adjacency_list<node_id_type, E> &&__forward) {
    std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > fwd(
            new unidirectional_adjacency_list<node_id_type, E>(std::move(__forward)));
    return make_bidirectional_undirected(fwd);
}

template<typename E>
adjacency_list<E>
adjacency_list<E>::make_bidirectional_undirected(
        const std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > &__edges) {
    std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > forward(__edges);
    std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > backward(__edges);

    return adjacency_list<E>(forward, backward);
}

template<typename E>
adjacency_list<E>
adjacency_list<E>::make_bidirectional(unidirectional_adjacency_list<node_id_type, E> &&__forward) {
    std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > fwd(
            new unidirectional_adjacency_list<node_id_type, E>(std::move(__forward)));
    return make_bidirectional(fwd);
}

template<typename E>
adjacency_list<E>
adjacency_list<E>::make_bidirectional(const std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > &__forward) {
    std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > forward(__forward);
    std::shared_ptr<unidirectional_adjacency_list<node_id_type, E> > backward(
            new unidirectional_adjacency_list<node_id_type, E>(__forward->inverse()));

    return adjacency_list<E>(forward, backward);
}

template<typename E>
adjacency_list<E>::adjacency_list(std::shared_ptr<const unidirectional_adjacency_list<node_id_type, E> > __forward,
                                  std::shared_ptr<unidirectional_adjacency_list<node_id_type, E> > __backward)
        : m_forward(__forward), m_backward(__backward)
{}

template<typename E>
bool
adjacency_list<E>::operator==(const adjacency_list<E> &__other) {
    return m_forward == __other.m_forward;
}

template<typename E>
size_t
adjacency_list<E>::node_count() const {
    return m_forward->node_count();
}

template<typename E>
size_t
adjacency_list<E>::edge_count() const {
    return m_forward->edge_count();
}

template<typename E>
adjacency_list<E>
adjacency_list<E>::inverse() const {
    return adjacency_list<E>(m_backward, m_forward);
}