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
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
dijkstra<G, Q, UseEdge, L>::dijkstra(dijkstra<G, Q, UseEdge, L> &&__other) noexcept
        : _M_labels(std::move(__other._M_labels)),
          _M_graph(std::move(__other._M_graph)),
          _M_queue(std::move(__other._M_queue)) {
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
dijkstra<G, Q, UseEdge, L>::node_cost_pair_type
dijkstra<G, Q, UseEdge, L>::current() const {
    return _M_queue.empty() ? none_value<dijkstra<G, Q, UseEdge, L>::node_cost_pair_type> : _M_queue.top();
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
bool
dijkstra<G, Q, UseEdge, L>::reached(G::node_id_type __node) const {
    return _M_labels.reached(__node);
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
bool
dijkstra<G, Q, UseEdge, L>::queue_empty() const {
    return _M_queue.empty();
}

template<RoutableGraph G, DijkstraQueue<G> Q,
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
void
dijkstra<G, Q, UseEdge, L>::init(node_id_type __start_node, node_id_type __target_node) {
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
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
void
dijkstra<G, Q, UseEdge, L>::expand(node_cost_pair_type __node) {
    assert(!is_none(__node.node));

    static std::vector<node_cost_pair_type> node_cost_pairs;


    if constexpr (
            requires(G &&t, node_cost_pair_type n) { t.outgoing_edges(n.node, n.predecessor, 1.0F); }
            ) {

        auto &&edges = _M_graph.topology().outgoing_edges(__node.node, __node.predecessor, M_PI / 6);
        for (auto edge: edges) {
            // ignore certain edges
            if (!_M_use_edge(__node.node, edge)) [[unlikely]] {
                continue;
            }

            assert (!is_none(edge.destination));
            // assert (_M_graph.has_edge(__node.node, edge.destination));

            const node_id_type successor = edge.destination;
            const distance_t successor_cost = _M_labels.get(successor).distance; // use shortest distance
            const distance_t new_cost = edge.info.cost + __node.distance;

            assert(new_cost >= __node.distance);
            if (new_cost < successor_cost) [[likely]] {
                assert (edge.destination != __node.predecessor);

                // (re-)insert node into the queue with updated priority
                node_cost_pairs.emplace_back(successor, __node.node, new_cost);

                // label current node with preliminary value
                _M_labels.label(successor, node_cost_pairs.back());
            }
        }
    } else {
        auto &&edges = _M_graph.topology().outgoing_edges(__node.node, __node.predecessor);
        for (auto edge: edges) {
            // ignore certain edges
            if (!_M_use_edge(__node.node, edge)) [[unlikely]] {
                continue;
            }

            assert (!is_none(edge.destination));
            // assert (_M_graph.has_edge(__node.node, edge.destination));

            const node_id_type successor = edge.destination;
            const distance_t successor_cost = _M_labels.get(successor).distance; // use shortest distance
            const distance_t new_cost = edge.info.cost + __node.distance;

            assert(new_cost >= __node.distance);
            if (new_cost < successor_cost) [[likely]] {
                assert (edge.destination != __node.predecessor);

                // (re-)insert node into the queue with updated priority
                node_cost_pairs.emplace_back(successor, __node.node, new_cost);

                // label current node with preliminary value
                _M_labels.label(successor, node_cost_pairs.back());
            }
        }
    }

    // push all
    _M_queue.push_range({node_cost_pairs.begin(), node_cost_pairs.end()});

    node_cost_pairs.clear();
}


template<RoutableGraph G, DijkstraQueue<G> Q,
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
void
dijkstra<G, Q, UseEdge, L>::step() {
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


template<RoutableGraph G, DijkstraQueue<G> Q, EdgePredicate<G> UseEdge, DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
typename G::distance_type dijkstra<G, Q, UseEdge, L>::min_path_length() const {
    return current().min_distance();
}
