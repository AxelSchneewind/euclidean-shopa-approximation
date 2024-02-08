#pragma once

#include "adjacency_list.h"
#include "graph_properties.h"

#include <memory>


template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional_undirected(unidirectional_adjacency_list<NodeId, E> &&forward) {
    assert(graph_properties::is_bidirectional(forward));

    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> fwd(
            new unidirectional_adjacency_list<NodeId, E>(std::move(forward)));
    return make_bidirectional_undirected(fwd);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional_undirected(
        std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> edges) {
    assert(graph_properties::is_bidirectional(*edges));

    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> forward(edges);
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> backward(edges);

    return adjacency_list<NodeId, E>(forward, backward);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional(unidirectional_adjacency_list<NodeId, E> &&forward) {
    std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> fwd(
            new unidirectional_adjacency_list<NodeId, E>(std::move(forward)));
    return make_bidirectional(fwd);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::make_bidirectional(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> forward) {
    std::shared_ptr<unidirectional_adjacency_list<NodeId, E>> backward { std::make_shared<unidirectional_adjacency_list<NodeId, E>>(forward->inverse()) };

    return adjacency_list<NodeId, E>(forward, backward);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> forward,
                                          std::shared_ptr<unidirectional_adjacency_list<NodeId, E>> backward)
        : _forward(forward), _backward(backward) {
}

template<typename NodeId, typename E>
bool
adjacency_list<NodeId, E>::operator==(const adjacency_list<NodeId, E> &other) {
    return _forward == other._forward;
}

template<typename NodeId, typename E>
size_t
adjacency_list<NodeId, E>::node_count() const {
    return _forward->node_count();
}

template<typename NodeId, typename E>
size_t
adjacency_list<NodeId, E>::edge_count() const {
    return _forward->edge_count();
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>
adjacency_list<NodeId, E>::invert(const adjacency_list<NodeId, E> &other) {
    return adjacency_list<NodeId, E>(other._backward, other._forward);
}


template<typename NodeId, typename E>
bool adjacency_list<NodeId, E>::has_edge(NodeId source, NodeId destination) const {
    return _forward->has_edge(source, destination);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::edge_id_type
adjacency_list<NodeId, E>::edge_id(NodeId source, NodeId destination) const {
    return _forward->edge_id(source, destination);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::edge_id_type
adjacency_list<NodeId, E>::edge_id(NodeId source) const {
    return _forward->edge_id(source);
}


template<typename NodeId, typename E>
E adjacency_list<NodeId, E>::edge(adjacency_list::edge_id_type id) const { return _forward->edge(id); }

template<typename NodeId, typename E>
NodeId adjacency_list<NodeId, E>::destination(adjacency_list::edge_id_type id) const {
    return _forward->destination(id);
}

template<typename NodeId, typename E>
NodeId adjacency_list<NodeId, E>::source(adjacency_list::edge_id_type id) const {
    return _forward->source(id);
}

template<typename NodeId, typename E>
counter<NodeId> adjacency_list<NodeId, E>::node_ids() const {
    return {(NodeId) node_count()};
}

template<typename NodeId, typename E>
counter<typename adjacency_list<NodeId, E>::edge_id_type> adjacency_list<NodeId, E>::edge_ids() const {
    return {(edge_id_type) edge_count()};
}

template<typename NodeId, typename E>
std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
adjacency_list<NodeId, E>::incoming_edges(NodeId destination, NodeId from) const {
    return _backward->outgoing_edges(destination);
}

template<typename NodeId, typename E>
std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
adjacency_list<NodeId, E>::incoming_edges(NodeId destination) const {
    return _backward->outgoing_edges(destination);
}

template<typename NodeId, typename E>
std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
adjacency_list<NodeId, E>::outgoing_edges(NodeId source, NodeId from) const {
    return _forward->outgoing_edges(source);
}

template<typename NodeId, typename E>
std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
adjacency_list<NodeId, E>::outgoing_edges(NodeId source) const {
    return _forward->outgoing_edges(source);
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(adjacency_list<NodeId, E> &&other) noexcept
        : _forward(std::move(other._forward)), _backward(std::move(other._backward)) {
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(const adjacency_list<NodeId, E> &other) noexcept
        : _forward(other._forward), _backward(other._backward) {
}

template<typename NodeId, typename E>
adjacency_list<NodeId, E>::adjacency_list(
        const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &forward,
        const std::shared_ptr<const unidirectional_adjacency_list<NodeId, E>> &backward)
        : _forward(forward), _backward(backward) {
}
