#pragma once

#include "unidirectional_adjacency_list.h"

#include "../util/list_intersection.h"
#include "../util/remove_duplicates.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>


template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_node(NodeId __node) {
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

    adjacency_list_edge<NodeId, E> edge(__source, __destination, __info);
    _M_edges.emplace_back(edge);
    ++_M_edge_count;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edge(NodeId __source, NodeId __destination) {
    add_node(__source);
    add_node(__destination);

    adjacency_list_edge<NodeId, E> edge(__source, __destination);
    _M_edges.emplace_back(edge);
    ++_M_edge_count;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges(const std::vector<edge_type> &edges) {
    for (auto& e : edges)
        _M_edges.emplace_back(e);
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges(std::vector<edge_type> &&edges) {
    while(!edges.empty()){
        _M_edges.emplace_back(edges.back());
        edges.pop_back();
        edges.shrink_to_fit();
    }
}


template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges_from_triangulation( std::vector<std::array<node_id_type, 3>> const&faces) {
    for (auto& tri : faces) {
        for (int i = 0; i < 3; ++i) {
            int next = (i + 1) % 3;

            if (tri[i] < tri[next])
                add_edge(tri[i], tri[next]);
            else
                add_edge(tri[next], tri[i]);
        }
    }
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges_from_triangulation(
        std::vector<std::array<node_id_type, 3>> &&faces) {
    while(!faces.empty()) {
        auto tri = faces.back();
        for (int i = 0; i < 3; ++i) {
            int next = (i + 1) % 3;

            if (tri[i] < tri[next])
                add_edge(tri[i], tri[next]);
            else
                add_edge(tri[next], tri[i]);
        }
        faces.pop_back();
        faces.shrink_to_fit();
    }

    faces.clear();
    std::move(faces);
}


template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::insert_backward_edges() {
    std::size_t edge_count = _M_edges.size();

    for (std::size_t i = 0; i < edge_count; ++i) {
        auto edge = _M_edges[i];
        std::swap(edge.source, edge.destination);
        _M_edges.emplace_back(edge);
    }

    _M_edge_count *= 2;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::sort_edges() {
    // order by source id
    auto order = [&](adjacency_list_edge<NodeId, E> __e1, adjacency_list_edge<NodeId, E> __e2) {
        return __e1.source < __e2.source || (__e1.source == __e2.source && __e1.destination < __e2.destination);
    };
    std::sort(_M_edges.begin(), _M_edges.end(), order);
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::remove_duplicates() {
    sort_edges();
    remove_duplicates_sorted(_M_edges);
}


template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::get() {
    sort_edges();

    // remove duplicates
    remove_duplicates_sorted<adjacency_list_edge<NodeId, E>>(_M_edges);
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
inline NodeId
unidirectional_adjacency_list<NodeId, E>::source(edge_id_t __edge) const {
    assert(contains_edge(__edge));
    return _M_sources[__edge];
}

template<typename NodeId, typename E>
inline NodeId
unidirectional_adjacency_list<NodeId, E>::destination(edge_id_t __edge) const {
    assert(contains_edge(__edge));
    return _M_edges[__edge].destination;
}


template<typename NodeId, typename E>
inline unidirectional_adjacency_list<NodeId, E>::edge_index_type
unidirectional_adjacency_list<NodeId, E>::edge_id(NodeId __source, NodeId __dest) const {
    assert(contains_node(__source));
    assert(contains_node(__dest));

    edge_id_t result = none_value<edge_index_type>;

    for (int idx = _M_offsets[__source]; idx < _M_offsets[__source + 1]; ++idx) {
        if (_M_edges[idx].destination == __dest)
            result = idx;
    }

    return result;
}


template<typename NodeId, typename E>
inline unidirectional_adjacency_list<NodeId, E>::edge_index_type
unidirectional_adjacency_list<NodeId, E>::edge_id(NodeId __source) const {
    assert(contains_node(__source));

    edge_id_t result = _M_offsets[__source];
    return result;
}

template<typename NodeId, typename E>
inline bool
unidirectional_adjacency_list<NodeId, E>::has_edge(NodeId __source, NodeId __dest) const {
    return !is_none(edge_id(__source, __dest));
}

template<typename NodeId, typename E>
inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
unidirectional_adjacency_list<NodeId, E>::outgoing_edges(NodeId __node) const {
    assert(contains_node(__node));
    assert(contains_edge(offset(__node)) || offset(__node) == edge_count());

    auto first = _M_edges.begin() + (size_t) offset(__node);
    auto last = _M_edges.begin() + (size_t) offset_next(__node);
    return std::span(first, last);
}

template<typename NodeId, typename E>
inline edge_id_t
unidirectional_adjacency_list<NodeId, E>::offset(NodeId __node) const {
    assert(contains_node(__node));

    return _M_offsets[__node];
}

template<typename NodeId, typename E>
inline edge_id_t
unidirectional_adjacency_list<NodeId, E>::offset_next(NodeId __node) const {
    assert(contains_node(__node));

    return _M_offsets[__node + 1];
}

template<typename NodeId, typename E>
inline E
unidirectional_adjacency_list<NodeId, E>::edge(edge_id_t __edge) const {
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
        ++index;
    }

    while (_M_offsets.size() <= __node_count) {
        _M_offsets.emplace_back(_M_edge_count);
    }

    // invert order of vector
    for (size_t i = 0; i < __edges.size() / 2; ++i) {
        std::swap(__edges[i], __edges[__edges.size() - i - 1]);
    }

    // split edges into source array and dest/info-array
    // TODO make faster/inplace
    int count = 0;
    while (!__edges.empty()) {
        auto full_edge = __edges.back();
        _M_sources.push_back(full_edge.source);
        _M_edges.emplace_back(full_edge.destination, full_edge.info);
        __edges.pop_back();
        if (count % (4 * 1024 * 1024) == 0) {
            __edges.shrink_to_fit();
        }
        ++count;
    }
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


template<typename NodeId, typename E>
struct std::hash<adjacency_list_edge<NodeId, E>> {
    std::size_t operator()(const adjacency_list_edge<NodeId, E> &__s) const noexcept {
        std::size_t hash1 = std::hash<NodeId>{}(__s.source);
        std::size_t hash2 = std::hash<NodeId>{}(__s.destination);
        return hash1 ^ (hash2 << 1);
    }
};
