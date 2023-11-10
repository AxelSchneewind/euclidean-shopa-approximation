#pragma once

#include "../util/list_intersection.h"
#include "unidirectional_adjacency_list.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>
#include "../util/remove_duplicates.h"


template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_node(const NodeId &__node) {
    if (_M_node_count <= (std::size_t) __node) {
        _M_node_count = (std::size_t) __node + 1;
    }
}

template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edge(NodeId __source,
                                                                           NodeId __destination,
                                                                           E __info) {
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
        return __e1.source < __e2.source || (__e1.source == __e2.source && __e1.destination < __e2.destination);
    };
    std::sort(_M_edges.begin(), _M_edges.end(), order);

    // remove duplicates
    // TODO removing duplicates somehow breaks everything
    //remove_duplicates_sorted<adjacency_list_edge<NodeId, E>>(_M_edges);
    _M_edge_count = _M_edges.size();

    auto result = unidirectional_adjacency_list<NodeId, E>(_M_node_count, std::move(_M_edges));

    // reset
    _M_node_count = 0;
    _M_edge_count = 0;
    _M_edges.clear();

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
bool unidirectional_adjacency_list<NodeId, E>::contains_node(node_id_type __node_id) const {
    return !is_none(__node_id) && __node_id >= 0 && __node_id < node_count();
}


template<typename NodeId, typename E>
bool unidirectional_adjacency_list<NodeId, E>::contains_edge(
        unidirectional_adjacency_list::edge_index_type __edge_index) const {
    return !is_none(__edge_index) && __edge_index >= 0 && __edge_index < edge_count();
}


template<typename NodeId, typename E>
inline const NodeId &
unidirectional_adjacency_list<NodeId, E>::source(const edge_id_t &__edge) const {
    assert(contains_edge(__edge));
    return _M_sources[__edge];
}

template<typename NodeId, typename E>
inline const NodeId &
unidirectional_adjacency_list<NodeId, E>::destination(const edge_id_t &__edge) const {
    assert(contains_edge(__edge));
    return _M_edges[__edge].destination;
}

template<typename NodeId, typename E>
inline NodeId &
unidirectional_adjacency_list<NodeId, E>::destination(const edge_id_t &__edge) {
    assert(contains_edge(__edge));
    return _M_edges[__edge].destination;
}

template<typename NodeId, typename E>
inline unidirectional_adjacency_list<NodeId, E>::edge_index_type
unidirectional_adjacency_list<NodeId, E>::edge_index(const NodeId &__source, const NodeId &__dest) const {
    assert(contains_node(__source));
    assert(contains_node(__dest));

    edge_id_t idx = _M_offsets[__source];
    while (idx < _M_offsets[__source + 1] && destination(idx) != __dest) {
        idx++;
    }

    return idx != _M_offsets[__source + 1] ? idx : none_value<edge_index_type>();
}

template<typename NodeId, typename E>
inline bool
unidirectional_adjacency_list<NodeId, E>::has_edge(const NodeId &__source, const NodeId &__dest) const {
    assert(contains_node(__source));
    assert(contains_node(__dest));

    edge_id_t idx = _M_offsets[__source];
    while (idx < _M_offsets[__source + 1] && destination(idx) != __dest) {
        idx++;
    }

    return (idx < _M_offsets[__source + 1]);
}

template<typename NodeId, typename E>
inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
unidirectional_adjacency_list<NodeId, E>::outgoing_edges(const NodeId &__node) const {
    assert(contains_node(__node));
    assert(contains_edge(offset(__node)) || offset(__node) == edge_count());

    auto first = _M_edges.begin() + (size_t) offset(__node);
    auto last = _M_edges.begin() + (size_t) offset_next(__node);
    return std::span(first, last);
}

template<typename NodeId, typename E>
inline const edge_id_t &
unidirectional_adjacency_list<NodeId, E>::offset(const NodeId &__node) const {
    assert(contains_node(__node));

    return _M_offsets[__node];
}

template<typename NodeId, typename E>
inline const edge_id_t &
unidirectional_adjacency_list<NodeId, E>::offset_next(const NodeId &__node) const {
    assert(contains_node(__node));

    return _M_offsets[__node + 1];
}

template<typename NodeId, typename E>
inline const E &
unidirectional_adjacency_list<NodeId, E>::edge(const edge_id_t &__edge) const {
    assert(contains_edge(__edge));

    return _M_edges[__edge].info;
}

template<typename NodeId, typename E>
inline E &
unidirectional_adjacency_list<NodeId, E>::edge(const edge_id_t &__edge) {
    assert(contains_edge(__edge));

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
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(size_t __node_count,
                                                                        std::vector<adjacency_list_edge<NodeId, E>> &&__edges)
        : _M_node_count(__node_count), _M_edge_count(__edges.size()) {

    // make offset and source arrays
    int index = 0;
    for (auto edge: __edges) {
        while (_M_offsets.size() <= edge.source) {
            _M_offsets.emplace_back(index);
        }
        index++;
    }

    while (_M_offsets.size() <= __node_count) {
        _M_offsets.emplace_back(_M_edge_count);
    }

    // split edges into source array and dest/info-array
    // TODO make inplace
    _M_edges.resize(_M_edge_count);
    _M_sources.reserve(_M_node_count);
    std::transform(__edges.begin(), __edges.end(), _M_edges.begin(),
                   [&](adjacency_list_edge<NodeId, E> __edge1) {
                       this->_M_sources.push_back(__edge1.source);
                       return internal_adjacency_list_edge<NodeId, E>{__edge1.destination, __edge1.info};
                   });

    __edges.clear();
    std::move(__edges);
}

template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(std::vector<edge_id_t> &&__offsets,
                                                                        std::vector<NodeId> &&__sources,
                                                                        std::vector<internal_adjacency_list_edge<NodeId, E>> &&__edges)
        : _M_node_count(__offsets.size() - 1),
          _M_edge_count(__sources.size()),
          _M_offsets(std::move(__offsets)),
          _M_sources(std::move(__sources)),
          _M_edges(std::move(__edges)) {
}


template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(
        unidirectional_adjacency_list &&__other) noexcept
        : _M_node_count(__other._M_node_count),
          _M_edge_count(__other._M_edge_count),
          _M_offsets(std::move(__other._M_offsets)),
          _M_sources(std::move(__other._M_sources)),
          _M_edges(std::move(__other._M_edges)) {}

// TODO refactor using adjacency_list_builder
template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>
unidirectional_adjacency_list<NodeId, E>::inverse() const {
    adjacency_list_builder builder(node_count());

    for (size_t edge_index = 0; edge_index < edge_count(); edge_index++) {
        builder.add_edge(destination(edge_index), source(edge_index), edge(edge_index));
    }

    return builder.get();
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
