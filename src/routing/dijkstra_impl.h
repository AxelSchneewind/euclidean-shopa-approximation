#pragma once

#include "dijkstra.h"

#include "../graph/base_types.h"

#include <cassert>
#include <cstddef>
#include <cmath>


template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
typename L::label_type dijkstra<G, Q, L, N>::get_label(node_id_type node) const { return _labels.at(node); }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
L &dijkstra<G, Q, L, N>::labels() { return _labels; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
const N &dijkstra<G, Q, L, N>::neighbors() const { return _neighbors; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
const L &dijkstra<G, Q, L, N>::labels() const { return _labels; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
Q &dijkstra<G, Q, L, N>::queue() { return _queue; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
const Q &dijkstra<G, Q, L, N>::queue() const { return _queue; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
dijkstra<G, Q, L, N>::node_id_type dijkstra<G, Q, L, N>::target() const { return _target_node; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
dijkstra<G, Q, L, N>::node_id_type dijkstra<G, Q, L, N>::source() const { return _start_node; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
dijkstra<G, Q, L, N>::dijkstra(const G &graph, Q &&queue, L &&labels, N &&neighbors)
        : _graph(graph), _queue(std::move(queue)),
          _labels(std::move(labels)), _neighbors(std::move(neighbors)) {}

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
dijkstra<G, Q, L, N>::dijkstra(const G &graph, dijkstra &&other)
        : _graph(graph), _queue{std::move(other._queue)},
          _labels(_graph, std::move(other._labels)), _neighbors(_graph, _labels) {}

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
dijkstra<G, Q, L, N>::dijkstra(const G &graph)
        : _graph(graph), _queue{_graph}, _labels(_graph), _neighbors{_graph, _labels},
          _start_node{optional::none_value<node_id_type>}, _target_node{optional::none_value<node_id_type>} {}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
dijkstra<G, Q, L, N>::dijkstra(dijkstra<G, Q, L, N> &&other) noexcept
        : _labels(std::move(other._labels)),
          _graph(std::move(other._graph)),
          _queue(std::move(other._queue)),
          _neighbors(std::move(other._neighbors)) {
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
dijkstra<G, Q, L, N>::node_cost_pair_type
dijkstra<G, Q, L, N>::current() const {
    assert(!_queue.empty());
    return _queue.empty() ? optional::none_value<dijkstra<G, Q, L, N>::node_cost_pair_type> : _queue.top();
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
bool
dijkstra<G, Q, L, N>::reached(G::node_id_type node) const {
    return !optional::is_none(_labels.at(node).predecessor());
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
bool
dijkstra<G, Q, L, N>::queue_empty() const {
    return _queue.empty();
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
void
dijkstra<G, Q, L, N>::init(node_id_type start_node, node_id_type target_node) {
    _target_node = target_node;
    _start_node = start_node;

    _queue.init(start_node, target_node);
    _labels.init(start_node, target_node);

    // add start node to queue
    if (!optional::is_none(start_node)) {
        _queue.push(start_node, start_node, 0.0);
    }

    _pull_count = 0;
    _push_count = 0;
    _edges_checked = 0;
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
void
dijkstra<G, Q, L, N>::expand(node_cost_pair_type node) {
    static std::vector<node_cost_pair_type> node_cost_pairs;
    static std::vector<coordinate_t > coordinates;
    node_cost_pairs.clear();
    coordinates.clear();

    assert(!optional::is_none(node.node()));
    assert(node.node() == _start_node || node.node() != node.predecessor());

    static constexpr bool geometric_queue     = requires(Q q, std::span<node_cost_pair_type> out, std::span<coordinate_t> coords_out) { q.push_range(out, coords_out);};
    static constexpr bool geometric_neighbors = requires(N n, std::vector<node_cost_pair_type> out, std::vector<coordinate_t> coords_out) { n(out, coords_out);};
    if constexpr (geometric_queue && geometric_neighbors) {
        _neighbors(node, node_cost_pairs, coordinates);
    } else {
        _neighbors(node, node_cost_pairs);
    }

    _edges_checked += node_cost_pairs.size();

    int to_index = 0;
    for (int i = 0; i < node_cost_pairs.size(); i++) {
        auto const &successor = node_cost_pairs[i];
        node_id_type const &successor_node = successor.node();

        assert(!optional::is_none(successor_node));
        assert(successor.predecessor() == node.node());
        assert(successor.distance() > 0);
        assert (successor_node == _start_node || _graph.has_edge(successor.predecessor(), successor_node));

        const distance_t& successor_cost = _labels.at(successor_node).distance(); // use shortest distance
        const distance_t& new_cost = successor.distance();

        assert(new_cost >= node.distance());
        if (new_cost < successor_cost) [[likely]] {
            assert(successor_node != node.predecessor());

            // (re-)insert node into the queue with updated priority
            if constexpr (geometric_neighbors && geometric_queue) {
                coordinates[to_index] = coordinates[i];
            }
            node_cost_pairs[to_index++] = successor;

            // label current node with preliminary value
            assert (_graph.has_edge(successor.predecessor(), successor_node));
            _labels.label(successor_node, successor);
            _labels.at(successor_node).predecessor() = optional::none_value<typename G::node_id_type>;
        }
    }

    // push all
    node_cost_pairs.resize(to_index);
    if constexpr (!geometric_neighbors && geometric_queue) {
        coordinates.resize(to_index);

        // get coordinates
        for (int i = 0; i < node_cost_pairs.size(); ++i) {
            coordinates[i] = _graph.node(node_cost_pairs[i].node()).coordinates;
        }
    }

    if constexpr (geometric_queue) {
        _queue.push_range(std::span{node_cost_pairs.begin(), node_cost_pairs.end()},
                          std::span{coordinates.begin(), coordinates.end()});
    } else {
        _queue.push_range(std::span{node_cost_pairs.begin(), node_cost_pairs.end()});
    }

    _push_count += to_index;
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
void
dijkstra<G, Q, L, N>::step() {
    node_cost_pair_type ncp = current();

    // expand to adjacent nodes
    expand(ncp);

    // remove current node
    _queue.pop();
    _pull_count++;

    // label current node
    assert(optional::is_none(_labels.at(ncp.node()).predecessor()));
    assert(!reached(ncp.node()));
    _labels.label(ncp.node(), ncp);

    // remove already labelled nodes
    while (!_queue.empty() && !optional::is_none(_labels.at(_queue.top().node()).predecessor())) [[likely]] {
        _queue.pop();
    }
}


template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
G::path_type dijkstra<G, Q, L, N>::path(node_id_type target) const {
    typename G::node_id_type fwd_node = target;

    auto result = std::vector<typename G::node_id_type>();
    result.push_back(fwd_node);

    if (!reached(target))
        return {std::move(result)};


    while (!optional::is_none(fwd_node) && fwd_node != _start_node) {
        fwd_node = get_label(fwd_node).predecessor();

        if (optional::is_none(fwd_node)) break;

        result.push_back(fwd_node);
    }

    for (int i = 0; i < result.size() / 2; ++i) {
        std::swap(result[i], result[result.size() - 1 - i]);
    }

    remove_duplicates_sorted(result);

    return {std::move(result)};
}

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
G::subgraph_type dijkstra<G, Q, L, N>::shortest_path_tree(std::size_t max_node_count) const {
    std::vector<typename G::node_id_type> nodes;
    std::vector<typename G::edge_id_type> edges;

    if (!reached(_start_node))
        return {std::move(nodes), std::move(edges)};

    // add nodes and edges that have been visited
    auto &&visited = labels().all_visited();
    for (auto const &node_id: visited) {
        if (nodes.size() >= max_node_count || edges.size() >= max_node_count)
            break;

        if (!reached(node_id) || labels().at(node_id).distance() >= current().value())
            continue;

        nodes.emplace_back(node_id);
        typename G::node_id_type predecessor = labels().at(node_id).predecessor();

        if (optional::is_none(predecessor) || predecessor == node_id)
            continue;

        typename G::edge_id_type edge = _graph.topology().edge_id(predecessor, node_id);
        edges.emplace_back(edge);
    }

    remove_duplicates(nodes);
    remove_duplicates(edges);

    typename G::subgraph_type subgraph{std::move(nodes), std::move(edges)};
    filter_nodes(subgraph, [&](auto const &node) -> bool { return get_label(node).distance() < current().value(); });

    return subgraph;
}

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
std::size_t dijkstra<G, Q, L, N>::edges_checked() const { return _edges_checked; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
std::size_t dijkstra<G, Q, L, N>::pull_count() const { return _pull_count; }

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N>
std::size_t dijkstra<G, Q, L, N>::push_count() const { return _push_count; }
