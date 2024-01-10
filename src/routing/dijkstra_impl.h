#pragma once

#include "dijkstra.h"

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../graph/unidirectional_adjacency_list.h"
#include "../triangulation/steiner_labels.h"

#include <cassert>
#include <queue>
#include <vector>

#include <cmath>
#include <concepts>
#include <iostream>


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
dijkstra<G, Q, L, N, UseEdge>::dijkstra(dijkstra<G, Q, L, N, UseEdge> &&__other) noexcept
        : _M_labels(std::move(__other._M_labels)),
          _M_graph(std::move(__other._M_graph)),
          _M_queue(std::move(__other._M_queue)),
          _M_neighbors(std::move(__other._M_neighbors)),
          _M_use_edge(std::move(__other._M_use_edge)) {
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
dijkstra<G, Q, L, N, UseEdge>::node_cost_pair_type
dijkstra<G, Q, L, N, UseEdge>::current() const {
    return _M_queue.empty() ? none_value<dijkstra<G, Q, L, N, UseEdge>::node_cost_pair_type> : _M_queue.top();
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
bool
dijkstra<G, Q, L, N, UseEdge>::reached(G::node_id_type __node) const {
    return !is_none(_M_labels.get(__node).predecessor);
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
bool
dijkstra<G, Q, L, N, UseEdge>::queue_empty() const {
    return _M_queue.empty();
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
void
dijkstra<G, Q, L, N, UseEdge>::init(node_id_type __start_node, node_id_type __target_node) {
    _M_target_node = __target_node;
    _M_start_node = __start_node;

    _M_queue.init(__start_node, __target_node);
    _M_labels.init(__start_node, __target_node);

    // add start node to queue
    if (!is_none(__start_node)) {
        _M_queue.push(__start_node, __start_node, 0);
    }

    _pull_count = 0;
    _push_count = 0;
    _edges_checked = 0;
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
void
dijkstra<G, Q, L, N, UseEdge>::expand(node_cost_pair_type node) {
    static std::vector<node_cost_pair_type> node_cost_pairs;
    node_cost_pairs.clear();

    assert(!is_none(node.node));
    assert(node.node == _M_start_node || node.node != node.predecessor);
    _M_neighbors(node, node_cost_pairs);
    _edges_checked += node_cost_pairs.size();

    int to_index = 0;
    for (int i = 0; i < node_cost_pairs.size(); i++) {
        auto const& successor = node_cost_pairs[i];
        node_id_type const& successor_node = successor.node;

        assert (!is_none(successor_node));
        assert(successor.predecessor == node.node);
        assert (successor_node == _M_start_node || _M_graph.has_edge(successor.predecessor, successor_node));

        const distance_t successor_cost = _M_labels.get(successor_node).distance; // use shortest distance
        const distance_t& new_cost = successor.distance;

        assert(new_cost >= node.distance);
        if (new_cost < successor_cost) [[likely]] {
            assert(successor_node != node.predecessor);

            // (re-)insert node into the queue with updated priority
            node_cost_pairs[to_index++] = successor;

            // label current node with preliminary value
            assert (_M_graph.has_edge(successor.predecessor, successor_node));

            auto preliminary_label = successor;
            // preliminary_label.predecessor = none_value<typename G::node_id_type>;
            _M_labels.label(successor_node, preliminary_label);
        }
    }

    // push all
    node_cost_pairs.resize(to_index);
    _push_count += to_index;
    _M_queue.push_range({node_cost_pairs.begin(), node_cost_pairs.end()});
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
void
dijkstra<G, Q, L, N, UseEdge>::step() {
    node_cost_pair_type ncp = current();

    // expand to adjacent nodes
    expand(ncp);

    // remove current node
    _M_queue.pop();
    _pull_count += 1;

    // label current node
    _M_labels.label(ncp.node, ncp);

    // remove already settled nodes
    while (!_M_queue.empty() && reached(_M_queue.top().node) &&
           _M_queue.top().distance > _M_labels.get(_M_queue.top().node).distance) [[likely]] {
        _pull_count += 1;
        _M_queue.pop();
    }
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
typename G::distance_type dijkstra<G, Q, L, N, UseEdge>::min_path_length() const {
    return current().min_distance();
}



template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
G::path_type dijkstra<G, Q, L, N, UseEdge>::path(node_id_type target) const {
    typename G::node_id_type fwd_node = target;

    auto result = std::vector<typename G::node_id_type>();
    result.push_back(fwd_node);

    if (!reached(target))
        return {std::move(result)};


    while (!is_none(fwd_node) && fwd_node != _M_start_node) {
        fwd_node = get_label(fwd_node).predecessor;

        if (is_none(fwd_node)) break;

        result.push_back(fwd_node);
    }

    for (int i = 0; i < result.size() / 2; ++i) {
        std::swap(result[i], result[result.size() - 1 - i]);
    }

    remove_duplicates_sorted(result);

    return {std::move(result)};
}

template<RoutableGraph G, DijkstraQueue<G> Q, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L, NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
G::subgraph_type dijkstra<G, Q, L, N, UseEdge>::shortest_path_tree(int max_node_count) const {
    std::vector<typename G::node_id_type> nodes;
    std::vector<typename G::edge_id_type> edges;

    if (!reached(_M_start_node))
        return {std::move(nodes), std::move(edges)};

    // add nodes and edges that have been visited
    auto &&visited = labels().all_visited();
    for (auto const& node_id: visited) {
        if (nodes.size() >= max_node_count || edges.size() >= max_node_count)
            break;

        if (!reached(node_id) || labels().get(node_id).distance >= current().value())
            continue;

        nodes.emplace_back(node_id);
        typename G::node_id_type predecessor = labels().get(node_id).predecessor;

        if (is_none(predecessor) || predecessor == node_id)
            continue;

        typename G::edge_id_type edge = _M_graph.topology().edge_id(predecessor, node_id);
        edges.emplace_back(edge);
    }

    remove_duplicates(nodes);
    remove_duplicates(edges);

    typename G::subgraph_type subgraph {std::move(nodes), std::move(edges)};
    filter_nodes(subgraph, [&](auto const& node) -> bool { return get_label(node).distance < current().value(); });

    return subgraph;
}
