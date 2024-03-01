#pragma once

#include "unidirectional_adjacency_list.h"

#include "../util/remove_duplicates.h"

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <span>
#include <vector>


template<typename NodeId, typename E>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_node(NodeId node) {
    if (_node_count <= node) [[unlikely]] {
        _node_count = (std::size_t) node + 1;
        _offsets_valid = false;
    }
}

template<typename NodeId, typename E>
template<typename... Args>
void
unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edge(NodeId source,
                                                                           NodeId destination,
                                                                           Args&&... info_args) {
    add_node(source);
    add_node(destination);

    _edges.emplace_back(source, destination, std::forward<Args>(info_args)...);
    ++_edge_count;

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edge(NodeId source, NodeId destination) {
    add_node(source);
    add_node(destination);

    _edges.emplace_back(source, destination);
    ++_edge_count;

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges(const std::vector<edge_type> &edges) {
    for (auto &e: edges) {
        add_node(e.source);
        add_node(e.destination);
        _edges.emplace_back(e);
    }

    _offsets_valid = false;
    _edges_sorted = false;
}

template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges(std::vector<edge_type> &&edges) {
    while (!edges.empty()) {
        add_node(edges.back().source);
        add_node(edges.back().destination);
        _edges.emplace_back(edges.back());
        edges.pop_back();
        if (_edges.size() == _edges.capacity())
            edges.shrink_to_fit();
    }

    _offsets_valid = false;
    _edges_sorted = false;
}


template<typename NodeId, typename E>
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::add_edges_from_triangulation(
        std::vector<std::array<node_id_type, 3>> const &faces) {
    for (auto &tri: faces) {
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
    while (!faces.empty()) {
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
        if (_edges.size() == _edges.capacity())
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
    auto order = [](adjacency_list_edge<NodeId, E> const &e1, adjacency_list_edge<NodeId, E> const &e2) {
        return e1.source < e2.source || (e1.source == e2.source && e1.destination < e2.destination);
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
    for (auto &&edge: _edges) {
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
    for (auto &&edge: _edges) {
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
    assert(new_node_ids.size() >= _node_count);

    // count nodes
    _node_count = 0;
    for (auto &&id: new_node_ids)
        if (!optional::is_none(id))
            _node_count = std::max(_node_count, (std::size_t) id);
    _node_count++;

    // change node ids in each edge
    for (int i = 0; i < _edge_count; ++i) {
        _edges[i].source = new_node_ids[_edges[i].source];
        _edges[i].destination = new_node_ids[_edges[i].destination];
    }

    // remove edges containing removed nodes (nodes that have none_value<...> as the new id)
    for (int i = 0; i < _edge_count;) {
        if (optional::is_none(_edges[i].source) || optional::is_none(_edges[i].destination)) {
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
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::filter_edges(EdgePredicate &&edge_predicate) {
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
void unidirectional_adjacency_list<NodeId, E>::adjacency_list_builder::filter_nodes(NodePredicate &&node_predicate) {
    std::vector<NodeId> new_node_ids(_node_count);

    int j = 0;
    for (int i = 0; i < _node_count; ++i) {
        if (node_predicate(i)) {
            new_node_ids[i] = j++;
        } else {
            new_node_ids[i] = optional::none_value<NodeId>;
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
bool unidirectional_adjacency_list<NodeId, E>::contains_node(node_id_type node_id) const {
    return !optional::is_none(node_id) && node_id >= 0 && node_id < node_count();
}


template<typename NodeId, typename E>
bool unidirectional_adjacency_list<NodeId, E>::contains_edge(
        unidirectional_adjacency_list::edge_index_type edge_index) const {
    return !optional::is_none(edge_index) && edge_index >= 0 && edge_index < edge_count();
}


template<typename NodeId, typename E>
inline NodeId
unidirectional_adjacency_list<NodeId, E>::source(edge_id_t edge) const {
    assert(contains_edge(edge));
    return _M_sources[edge];
}

template<typename NodeId, typename E>
inline NodeId
unidirectional_adjacency_list<NodeId, E>::destination(edge_id_t edge) const {
    assert(contains_edge(edge));
    return _M_edges[edge].destination;
}


template<typename NodeId, typename E>
inline unidirectional_adjacency_list<NodeId, E>::edge_index_type
unidirectional_adjacency_list<NodeId, E>::edge_id(NodeId source, NodeId dest) const {
    assert(contains_node(source));
    assert(contains_node(dest));

    edge_id_t result = optional::none_value<edge_index_type>;

    for (int idx = _M_offsets[source]; idx < _M_offsets[source + 1]; ++idx) {
        if (_M_edges[idx].destination == dest)
            result = idx;
    }

    return result;
}


template<typename NodeId, typename E>
inline unidirectional_adjacency_list<NodeId, E>::edge_index_type
unidirectional_adjacency_list<NodeId, E>::edge_id(NodeId source) const {
    assert(contains_node(source));

    edge_id_t result = _M_offsets[source];
    return result;
}

template<typename NodeId, typename E>
inline bool
unidirectional_adjacency_list<NodeId, E>::has_edge(NodeId source, NodeId dest) const {
    return !optional::is_none(edge_id(source, dest));
}

template<typename NodeId, typename E>
inline std::span<const internal_adjacency_list_edge<NodeId, E>, std::dynamic_extent>
unidirectional_adjacency_list<NodeId, E>::outgoing_edges(NodeId node) const {
    assert(contains_node(node));
    assert(contains_edge(offset(node)) || offset(node) == edge_count());

    auto first = _M_edges.begin() + (size_t) offset(node);
    auto last = _M_edges.begin() + (size_t) offset_next(node);
    return std::span(first, last);
}

template<typename NodeId, typename E>
inline edge_id_t
unidirectional_adjacency_list<NodeId, E>::offset(NodeId node) const {
    assert(contains_node(node));

    return _M_offsets[node];
}

template<typename NodeId, typename E>
inline edge_id_t
unidirectional_adjacency_list<NodeId, E>::offset_next(NodeId node) const {
    assert(contains_node(node));

    return _M_offsets[node + 1];
}

template<typename NodeId, typename E>
inline E
unidirectional_adjacency_list<NodeId, E>::edge(edge_id_t edge) const {
    assert(contains_edge(edge));

    return _M_edges[edge].info;
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
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(size_t node_count,
                                                                        std::vector<adjacency_list_edge<NodeId, E>> && edges)
        : _M_node_count(node_count), _M_edge_count(edges.size()) {
    // make offset and source arrays
    int index = 0;
    for (auto&& edge: edges) {
        assert(!optional::is_none(edge.source));
        while (_M_offsets.size() <= edge.source) {
            _M_offsets.emplace_back(index);
        }
        ++index;
    }

    while (_M_offsets.size() <= node_count) {
        _M_offsets.emplace_back(_M_edge_count);
    }

    // invert order of vector
    for (size_t i = 0; i < edges.size() / 2; ++i) {
        std::swap(edges[i], edges[edges.size() - i - 1]);
    }

    // split edges into source array and dest/info-array
    // TODO make faster/inplace
    int count = 0;
    while (!edges.empty()) {
        auto full_edge = edges.back();
        _M_sources.push_back(full_edge.source);
        _M_edges.emplace_back(full_edge);
        edges.pop_back();
        if (count % (4 * 1024 * 1024) == 0) {
            edges.shrink_to_fit();
        }
        ++count;
    }
}

template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(std::vector<edge_id_t> &&offsets,
                                                                        std::vector<NodeId> &&sources,
                                                                        std::vector<internal_adjacency_list_edge<NodeId,
                                                                                E>> &&edges)
        : _M_node_count(offsets.size() - 1),
          _M_edge_count(sources.size()),
          _M_offsets(std::move(offsets)),
          _M_sources(std::move(sources)),
          _M_edges(std::move(edges)) {
}


template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>::unidirectional_adjacency_list(
        unidirectional_adjacency_list &&other) noexcept
        : _M_node_count(other._M_node_count),
          _M_edge_count(other._M_edge_count),
          _M_offsets(std::move(other._M_offsets)),
          _M_sources(std::move(other._M_sources)),
          _M_edges(std::move(other._M_edges)) {
    other._M_node_count = 0;
    other._M_edge_count = 0;
}

template<typename NodeId, typename E>
unidirectional_adjacency_list<NodeId, E>
unidirectional_adjacency_list<NodeId, E>::inverse() const {
    adjacency_list_builder builder(node_count());

    for (std::size_t edge_index = 0; edge_index < edge_count(); edge_index++) {
        if constexpr (std::is_same_v<E, void>) {
            builder.add_edge(destination(edge_index), source(edge_index));
        } else {
            builder.add_edge(destination(edge_index), source(edge_index), edge(edge_index));
        }
    }

    return builder.get();
}


template<typename NodeId, typename E>
bool
unidirectional_adjacency_list<NodeId, E>::operator==(const unidirectional_adjacency_list<NodeId, E> &other) {
    if (_M_node_count != other.node_count || _M_edge_count != other.edge_count) {
        return false;
    }

    for (NodeId i = 0; i < _M_node_count; i++) {
        if (_M_offsets[i] != other._M_offsets[i]) {
            return false;
        }
    }

    for (edge_id_t i = 0; i < _M_edge_count; i++) {
        if (_M_edges[i] != other._M_edges[i]) {
            return false;
        }
    }

    return true;
}


template<typename NodeId, typename E>
struct std::hash<adjacency_list_edge<NodeId, E>> {
    std::size_t operator()(const adjacency_list_edge<NodeId, E> &s) const noexcept {
        std::size_t hash1 = std::hash<NodeId>{}(s.source);
        std::size_t hash2 = std::hash<NodeId>{}(s.destination);
        return hash1 ^ (hash2 << 1);
    }
};
