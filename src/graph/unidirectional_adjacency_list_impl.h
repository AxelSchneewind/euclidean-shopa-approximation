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
    if (_node_count <= __node) [[unlikely]] {
        _node_count = (std::size_t) __node + 1;
        _offsets_valid = false;
    }
}

template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edge(NodeId __source,
                                                                           NodeId __destination,
                                                                           E __info) {
    add_node(__source);
    add_node(__destination);

    _edges.emplace_back(__source, __destination, __info);
    ++_edge_count;

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edge(NodeId __source, NodeId __destination) {
    add_node(__source);
    add_node(__destination);

    adjacency_list_edge<NodeId, E> edge(__source, __destination);
    _edges.emplace_back(edge);
    ++_edge_count;

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges(const std::vector<edge_type> &edges) {
    for (auto& e : edges) {
        add_node(e.source);
        add_node(e.destination);
        _edges.emplace_back(e);
    }

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges(std::vector<edge_type> &&edges) {
    while(!edges.empty()){
        add_node(edges.back().source);
        add_node(edges.back().destination);
        _edges.emplace_back(edges.back());
        edges.pop_back();
        edges.shrink_to_fit();
    }

    _offsets_valid = false;
    _edges_sorted = false;
}


template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges_from_triangulation( std::vector<std::array<node_id_type, 3>> const&faces) {
    for (auto& tri : faces) {
        for (int i = 0; i < 3; ++i) {
            int next = (i + 1) % 3;
            add_node(tri[i]);
            add_node(tri[next]);

            // add edge such that source < destination
            if (tri[i] < tri[next])
                add_edge(tri[i], tri[next]);
            else if (tri[i] > tri[next])
                add_edge(tri[next], tri[i]);
        }
    }

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges_from_triangulation(
        std::vector<std::array<node_id_type, 3>> &&faces) {
    while(!faces.empty()) {
        auto tri = faces.back();
        for (int i = 0; i < 3; ++i) {
            int next = (i + 1) % 3;
            add_node(tri[i]);
            add_node(tri[next]);

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

    _offsets_valid = false;
    _edges_sorted = false;
}


template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::insert_backward_edges() {
    std::size_t edge_count = _edges.size();

    for (std::size_t i = 0; i < edge_count; ++i) {
        auto edge = _edges[i];
        std::swap(edge.source, edge.destination);
        _edges.emplace_back(edge);
    }

    _edge_count *= 2;

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::sort_edges() {
    if (_edges_sorted) return;

    // order by source id
    auto order = [&](adjacency_list_edge<NodeId, E> const& __e1, adjacency_list_edge<NodeId, E> const& __e2) {
        return __e1.source < __e2.source || (__e1.source == __e2.source && __e1.destination < __e2.destination);
    };
    std::sort(_edges.begin(), _edges.end(), order);

    _edges_sorted = true;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::remove_duplicates() {
    sort_edges();
    remove_duplicates_sorted(_edges);
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::remove_unconnected_nodes() {
    sort_edges();
    make_offsets();

    std::vector<bool> connected(_node_count, false);
    for(auto&& edge: _edges) {
        connected[edge.source] = true;
        connected[edge.destination] = true;
    }

    filter_nodes([&](auto node_id) -> bool { return connected[node_id]; });
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::make_offsets() {
    if (_offsets_valid && _edges_sorted) return;

    _offsets.clear();
    sort_edges();

    // make offset and source arrays
    edge_index_type index = 0;
    for (auto&& edge: _edges) {
        while (_offsets.size() <= edge.source) {
            _offsets.emplace_back(index);
        }
        ++index;
    }

    while (_offsets.size() <= _node_count) {
        _offsets.emplace_back(_edge_count);
    }

    _offsets_valid = true;
}


template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::reorder_nodes(std::span<node_id_type> new_node_ids) {
    assert (new_node_ids.size() >= _node_count);

    // count nodes
    _node_count = 0;
    for (auto&& id : new_node_ids)
        if (!is_none(id))
            _node_count = std::max(_node_count, (std::size_t)id);
    _node_count++;

    // change node ids in each edge
    for (int i = 0; i < _edge_count; ++i) {
        _edges[i].source = new_node_ids[_edges[i].source];
        _edges[i].destination = new_node_ids[_edges[i].destination];
    }

    // remove edges containing removed nodes (nodes that have none_value<...> as the new id)
    for (int i = 0; i < _edge_count; ) {
        if (is_none(_edges[i].source) || is_none(_edges[i].destination)) {
            _edges[i] = _edges.back();
            _edges.pop_back();
        } else {
            i++;
        }
    }
    _edge_count = _edges.size();

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
template<std::predicate<typename unidirectional_adjacency_list<NodeId, E>::edge_info_type> EdgePredicate>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::filter_edges(EdgePredicate&& edge_predicate) {
    int j = 0;
    for (int i = 0; i < _edge_count; ++i) {
        if (edge_predicate(i)) {
            _edges[j++] = _edges[i];
        }
    }
    _edges.resize(j);
    _edge_count = _edges.size();

    _offsets_valid = false;
}

template<typename NodeId, typename E>
template<std::predicate<typename unidirectional_adjacency_list<NodeId, E>::node_id_type> NodePredicate>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::filter_nodes(NodePredicate&& node_predicate) {
    std::vector<NodeId> new_node_ids(_node_count);

    int j = 0;
    for (int i = 0; i < _node_count; ++i) {
        if (node_predicate(i)){
            new_node_ids[i] = j++;
        } else {
            new_node_ids[i] = none_value<NodeId>;
        }
    }

    reorder_nodes({new_node_ids.begin(), new_node_ids.end()});
}




template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::get() {
    sort_edges();
    remove_duplicates();
    _edge_count = _edges.size();

    auto result = unidirectional_adjacency_list<NodeId, E>(_node_count, std::move(_edges));

    // reset
    _node_count = 0;
    _edge_count = 0;
    _edges.clear();

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
        assert(!is_none(edge.source));
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
