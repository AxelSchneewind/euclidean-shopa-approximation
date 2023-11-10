#pragma once

#include "dijkstra.h"

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../graph/unidirectional_adjacency_list.h"

#include <queue>
#include <vector>

#include <cmath>
#include <concepts>
#include <iostream>


template<RoutableGraph G, DijkstraQueue<G> Q, typename U, DijkstraLabels L>
dijkstra<G, Q, U, L>::dijkstra(const std::shared_ptr<const G> &__graph,
                               const G::topology_type &__adj_list)
        : _M_topology(__adj_list),
          _M_graph(__graph),
          _M_labels(__graph.get()),
          _M_use_edge(__graph.get()),
          _M_queue(__graph) {
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
dijkstra<G, Queue, U, L>::dijkstra(dijkstra<G, Queue, U, L> &&__other) noexcept
        : _M_topology(__other._M_topology), _M_labels(std::move(__other._M_labels)),
          _M_graph(std::move(__other._M_graph)),
          _M_queue(std::move(__other._M_queue)) {}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
dijkstra<G, Queue, U, L>::node_cost_pair
dijkstra<G, Queue, U, L>::current() const {
    return _M_queue.top();
}


template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
bool
dijkstra<G, Queue, U, L>::reached(G::node_id_type __node) const {
    return _M_labels.reached(__node);
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
bool
dijkstra<G, Queue, U, L>::queue_empty() const {
    return _M_queue.empty();
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
void
dijkstra<G, Queue, U, L>::init(node_id_type __start_node, node_id_type __target_node) {
    _M_target_node = __target_node;
    _M_start_node = __start_node;

    _M_queue.init(__start_node, __target_node);
    _M_labels.init(__start_node, __target_node);

    // add start node to queue
    if (!is_none(__start_node)) {
        _M_queue.push(__start_node, __start_node, 0);
    }
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
void
dijkstra<G, Queue, U, L>::expand(const dijkstra<G, Queue, U, L>::node_id_type &__node) {
    assert(!is_none(__node));

    auto edges = _M_topology.outgoing_edges(__node);
    for (auto &edge: edges) {
        assert(!is_none(edge.destination));
        assert(_M_graph->has_edge(__node, edge.destination));

        // ignore certain edges
        if (!_M_use_edge(__node, edge)) {
            continue;
        }

        const typename G::node_id_type &successor = edge.destination;
        const distance_t successor_cost = _M_labels.distance(successor);
        const distance_t new_cost = _M_labels.distance(__node) + edge.info.cost;

        if (new_cost < successor_cost) {
            // (re-)insert node into the queue with updated priority
            _M_queue.push(successor, __node, new_cost);
        }
    }
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
void
dijkstra<G, Queue, U, L>::step() {
    // remove already settled nodes
    while (!_M_queue.empty() && reached(_M_queue.top().node)) {
        _M_queue.pop();
    }

    if (_M_queue.empty()) {
        return;
    }

    node_cost_pair ncp = current();

    // label current node
    _M_labels.label(ncp);

    // expand to adjacent nodes
    expand(ncp.node);

    // remove current node
    _M_queue.pop();
}
