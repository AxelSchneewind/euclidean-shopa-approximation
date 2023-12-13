#pragma once

#include "dijkstra.h"

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../graph/unidirectional_adjacency_list.h"
#include "../triangulation/steiner_labels.h"

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
    return _M_labels.reached(__node);
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
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
void
dijkstra<G, Q, L, N, UseEdge>::expand(node_cost_pair_type node) {
    static std::vector<node_cost_pair_type> node_cost_pairs;
    node_cost_pairs.clear();

    assert(!is_none(node.node));
    _M_neighbors(node, node_cost_pairs);

    for (auto edge: node_cost_pairs) {
        assert (!is_none(edge.node));
        // assert (_M_graph.has_edge(__node.node, edge.destination));

        const node_id_type& successor = edge.node;
        const distance_t successor_cost = _M_labels.get(successor).distance; // use shortest distance
        const distance_t new_cost = edge.distance;

        assert(new_cost >= node.distance);
        if (new_cost < successor_cost) [[likely]] {
            assert (edge.node != node.predecessor);

            // (re-)insert node into the queue with updated priority
            // node_cost_pairs.emplace_back(successor, __node.node, new_cost);
            _M_queue.push(edge.node, edge.predecessor, edge.distance); // TODO fix

            // label current node with preliminary value
            _M_labels.label(successor, edge);
        }
    }

    // // push all
    // _M_queue.push_range({node_cost_pairs.begin(), node_cost_pairs.end()});
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
void
dijkstra<G, Q, L, N, UseEdge>::step() {
    node_cost_pair_type ncp = current();

    // label current node
    _M_labels.label(ncp.node, ncp);

    // expand to adjacent nodes
    expand(ncp);

    // remove current node
    _M_queue.pop();

    // remove already settled nodes
    while (!_M_queue.empty() && reached(_M_queue.top().node) &&
           _M_queue.top().distance > _M_labels.get(_M_queue.top().node).distance) [[likely]] {
        _M_queue.pop();
    }
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
typename G::distance_type dijkstra<G, Q, L, N, UseEdge>::min_path_length() const {
    return current().min_distance();
}
