#pragma once

#include "../util/remove_duplicates.h"
#include "unidirectional_adjacency_list.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_node(const NodeId &__node) {
    if (_M_node_count <= __node) {
        _M_node_count = __node + 1;
    }
}

template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edge(const NodeId &__source,
                                                                           const NodeId &__destination,
                                                                           const E &__info) {
    add_node(__source);
    add_node(__destination);

    auto edge = adjacency_list_edge<NodeId, E>(__source, __destination, __info);
    _M_edges.push_back(edge);
    _M_edge_count++;
}

template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::get() {
    // order by source id
    auto order = [&](adjacency_list_edge<NodeId, E> __e1, adjacency_list_edge<NodeId, E> __e2) {
        return __e1.source < __e2.source;
    };
    std::sort(_M_edges.begin(), _M_edges.end(), order);

    // remove duplicates
    remove_duplicates<adjacency_list_edge<NodeId, E>>(_M_edges);
    _M_edge_count = _M_edges.size();

    // make _M_offsets and source arrays
    int index = 0;
    _M_offsets.clear();
    for (auto edge: _M_edges) {
        while (_M_offsets.size() <= edge.source) {
            _M_offsets.push_back(index);
        }
        index++;
    }

    while (_M_offsets.size() <= _M_node_count) {
        _M_offsets.push_back(_M_edge_count);
    }

    auto result = unidirectional_adjacency_list<NodeId, E>(_M_node_count, _M_edge_count, std::move(_M_offsets),
                                                           std::move(_M_edges));

    // reset
    _M_node_count = 0;
    _M_edge_count = 0;
    _M_edges.clear();
    _M_offsets.clear();

    return result;
}


template<typename NodeId, typename E>
inline size_t
unidirectional_adjacency_list<NodeId, E>::node_count() const {
    return _M_node_count;
}

template<typename NodeId, typename E>
inline size_t
unidirectional_adjacency_list<NodeId, E>::edge_count() const {
    return _M_edge_count;
}

template<typename NodeId, typename E>
inline const NodeId &
unidirectional_adjacency_list<NodeId, E>::source(const edge_id_t &__edge) const {
    return _M_sources[__edge];
}

template<typename NodeId, typename E>
inline const NodeId &
unidirectional_adjacency_list<NodeId, E>::destination(const edge_id_t &__edge) const {
    return _M_edges[__edge].destination;
}

template<typename NodeId, typename E>
inline NodeId &
unidirectional_adjacency_list<NodeId, E>::destination(const edge_id_t &__edge) {
    return _M_edges[__edge].destination;
}

template<typename NodeId, typename E>
inline unidirectional_adjacency_list<NodeId, E>::edge_index_type
unidirectional_adjacency_list<NodeId, E>::edge_index(const NodeId &__source, const NodeId &__dest) const {
    edge_id_t idx = _M_offsets[__source];
    while (idx < _M_offsets[__source + 1] && destination(idx) != __dest) {
        idx++;
    }

    return idx;
}

template<typename NodeId, typename E>
inline bool
unidirectional_adjacency_list<NodeId, E>::has_edge(const NodeId &__source, const NodeId &__dest) const {
    edge_id_t idx = _M_offsets[__source];
    while (idx < _M_offsets[__source + 1] && destination(idx) != __dest) {
        idx++;
    }

    return (idx < _M_offsets[__source + 1]);
}

template<typename NodeId, typename E>
inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
unidirectional_adjacency_list<NodeId, E>::outgoing_edges(const NodeId &__node) const {
    auto first = &_M_edges[offset(__node)];
    auto last = &_M_edges[offset(__node + 1)];
    return std::span(first, last);
}

template<typename NodeId, typename E>
inline const edge_id_t &
unidirectional_adjacency_list<NodeId, E>::offset(const NodeId &__node) const {
    return _M_offsets[__node];
}

template<typename NodeId, typename E>
inline const edge_id_t &
unidirectional_adjacency_list<NodeId, E>::offset_next(const NodeId &__node) const {
    return _M_offsets[__node + 1];
}

template<typename NodeId, typename E>
inline const E &
unidirectional_adjacency_list<NodeId, E>::edge(const edge_id_t &__edge) const {
    return _M_edges[__edge].info;
}

template<typename NodeId, typename E>
inline E &
unidirectional_adjacency_list<NodeId, E>::edge(const edge_id_t &__edge) {
    return _M_edges[__edge].info;
}

template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::~unidirectional_adjacency_list() {
    _M_offsets.clear();
    _M_offsets.shrink_to_fit();
    _M_edges.clear();
    _M_edges.shrink_to_fit();
    _M_offsets.emplace_back(0);
}

template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(size_t __node_count, size_t __edge_count,
                                                                        std::vector<edge_id_t> &&__offsets,
                                                                        std::vector<adjacency_list_edge<NodeId, E>> &&__edges)
        : _M_node_count(__node_count), _M_edge_count(__edge_count), _M_offsets(std::move(__offsets)) {
    // check _M_offsets array
    if (this->_M_offsets.size() <= __node_count) { // given array is too small
        throw;
    }

    // split edges into source array and dest/info-array
    // TODO make inplace
    _M_edges.resize(__edge_count);
    std::transform(__edges.begin(), __edges.end(), _M_edges.begin(),
                   [&](adjacency_list_edge<NodeId, E> __edge1) {
                       this->_M_sources.push_back(__edge1.source);
                       return internal_adjacency_list_edge<NodeId, E>{__edge1.destination, __edge1.info};
                   });
    std::move(__edges);
}

template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(size_t __node_count, size_t __edge_count,
                                                                        std::vector<edge_id_t> &&__offsets,
                                                                        std::vector<NodeId> &&__sources,
                                                                        std::vector<internal_adjacency_list_edge<NodeId, E>> &&__edges)
        : _M_node_count(__node_count), _M_edge_count(__edge_count), _M_offsets(std::move(__offsets)),
          _M_sources(std::move(__sources)),
          _M_edges(std::move(__edges)) {
    // check _M_offsets array
    if (this->_M_offsets.size() <= __node_count) { // given array is too small
        throw;
    }
}


template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(
        unidirectional_adjacency_list &&__other) noexcept
        : _M_node_count(__other._M_node_count), _M_edge_count(__other._M_edge_count),
          _M_offsets(std::move(__other._M_offsets)),
          _M_edges(std::move(__other._M_edges)), _M_sources(std::move(__other._M_sources)) {}

// TODO refactor using adjacency_list_builder
template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>
unidirectional_adjacency_list<NodeId, E>::inverse() const {
    std::vector<std::vector<edge_id_t>> incoming_edges(node_count());

    for (size_t edge_index = 0; edge_index < edge_count(); edge_index++) {
        const internal_adjacency_list_edge<NodeId, E> &edge = _M_edges[edge_index];
        // insert nodes in inverse order
        incoming_edges[node_count() - edge.destination - 1].push_back(edge_index);
    }

    std::vector<edge_id_t> inv_offsets;
    std::vector<internal_adjacency_list_edge<NodeId, E>> inv_edges;

    std::vector<NodeId> inv_sources;

    // store new indices for each edge
    std::vector<edge_id_t> backward_ids;
    backward_ids.resize(edge_count());

    edge_id_t current = 0;
    for (size_t node_index = 0; node_index < node_count(); node_index++) {
        auto incoming = incoming_edges[node_count() - node_index - 1];
        for (edge_id_t edge: incoming) {
            // forward_ids[current] = edge;
            backward_ids[edge] = current++;
        }
    }

    for (size_t node_index = 0; node_index < node_count(); node_index++) {
        inv_offsets.push_back(inv_edges.size());

        auto incoming = incoming_edges.back();
        for (edge_id_t edge_id: incoming) {
            internal_adjacency_list_edge<NodeId, E> edge = _M_edges[edge_id];
            inv_sources.push_back(edge.destination);

            edge.destination = _M_sources[edge_id];
            inv_edges.push_back(edge);
        }

        incoming_edges.pop_back();
    }

    while (inv_offsets.size() <= node_count()) {
        inv_offsets.push_back(edge_count());
    }

    return unidirectional_adjacency_list(node_count(), edge_count(), std::move(inv_offsets), std::move(inv_sources),
                                         std::move(inv_edges));
}

template<typename NodeId, typename E>
bool
unidirectional_adjacency_list<NodeId, E>::operator==(const unidirectional_adjacency_list<NodeId, E> &__other) {
    if (_M_node_count != __other.node_count || _M_edge_count != __other.edge_count) {
        return false;
    }

    for (NodeId i = 0; i < _M_node_count; i++) {
        if (_M_offsets[i] != __other._M_offsets[i]) {
            return false;
        }
    }

    for (edge_id_t i = 0; i < _M_edge_count; i++) {
        if (_M_edges[i] != __other._M_edges[i]) {
            return false;
        }
    }

    return true;
}


template<template<typename> class A, typename E>
struct std::hash<A<E>>;

template<typename NodeId, typename E>
struct std::hash<adjacency_list_edge<NodeId, E>> {
    std::size_t operator()(const adjacency_list_edge<NodeId, E> &__s) const noexcept {
        std::size_t hash1 = std::hash<NodeId>{}(__s.source);
        std::size_t hash2 = std::hash<NodeId>{}(__s.destination);
        return hash1 ^ (hash2 << 1);
    }
};
