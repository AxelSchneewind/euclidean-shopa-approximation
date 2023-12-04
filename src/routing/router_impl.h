#pragma once

#include <cassert>
#include <exception>

#include "router.h"

#include "../graph/graph_impl.h"
#include "dijkstra_impl.h"
#include <deque>
#include <map>
#include <ostream>

#include "dijkstra_queues.h"

template<typename Graph, typename Dijkstra>
Graph::distance_type
router<Graph, Dijkstra>::min_route_distance(Dijkstra::node_cost_pair_type __node) const {
    return __node.min_distance();
}

template<typename Graph, typename Dijkstra>
Graph::subgraph_type
router<Graph, Dijkstra>::shortest_path_tree() const {
    std::vector<typename Graph::node_id_type> nodes;
    std::vector<typename Graph::edge_id_type> edges;

    // add nodes and edges of forward dijkstra
    auto &&visited = _M_forward_search.labels().all_visited();
    for (auto node_id: visited) {
        if (!_M_forward_search.reached(node_id))
            continue;

        nodes.push_back(node_id);
        typename Graph::node_id_type pred = _M_forward_search.labels().get(node_id).predecessor;

        if (is_none(pred) || !_M_forward_search.reached(pred) || pred == node_id) continue;

        typename Graph::edge_id_type edge = _M_graph.topology().edge_id(pred, node_id);
        edges.push_back(edge);
    }

    // add nodes and edges of backward dijkstra
    auto &&bwd_visited = _M_backward_search.labels().all_visited();
    for (auto node_id: bwd_visited) {
        if (!_M_backward_search.reached(node_id))
            continue;

        nodes.push_back(node_id);

        typename Graph::node_id_type succ = _M_backward_search.labels().get(node_id).predecessor;

        if (is_none(succ) || !_M_backward_search.reached(succ) || succ == node_id) continue;

        typename Graph::edge_id_type edge = _M_graph.topology().edge_id(node_id, succ);
        edges.push_back(edge);
    }

    remove_duplicates(nodes);
    remove_duplicates(edges);

    return {_M_graph, std::move(nodes), std::move(edges)};
}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(Graph const &__graph)
        : _M_graph(__graph),
          _M_forward_search(_M_graph),
          _M_backward_search(_M_graph),
          _M_start_node(none_value<typename Graph::node_id_type>),
          _M_target_node(none_value<typename Graph::node_id_type>),
          _M_mid_node(none_value<typename Graph::node_id_type>) {}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(router &&__routing) noexcept
        : _M_graph(__routing._M_graph), _M_forward_search(std::move(__routing._M_forward_search)),
          _M_backward_search(std::move(__routing._M_backward_search)),
          _M_start_node(__routing._M_start_node), _M_target_node(__routing._M_target_node),
          _M_mid_node(__routing._M_mid_node) {}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::step_forward() {
    assert (!_M_forward_search.queue_empty());

    typename Dijkstra::node_cost_pair_type current = _M_forward_search.current();
    assert(!is_none(current.node));

    _M_forward_search.step();

    // check if searches met and provide best result so far
    if (_M_backward_search.reached(current.node) && distance(current.node) < distance()) {
        _M_mid_node = current.node;
    }
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::step_backward() {
    assert (!_M_backward_search.queue_empty());

    typename Dijkstra::node_cost_pair_type current = _M_backward_search.current();
    assert(!is_none(current.node));

    _M_backward_search.step();
    assert(!is_none(current.node));

    // check if searches met and provide best result yet
    if (_M_forward_search.reached(current.node) && distance(current.node) < distance()) {
        _M_mid_node = current.node;
    }
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::compute_route() {
    bool done = false;
    while (!done) {
        auto dist = distance();
        bool const fwd_done =
                _M_forward_search.queue_empty() || min_route_distance(_M_forward_search.current()) > dist;
        bool const bwd_done =
                _M_backward_search.queue_empty() || min_route_distance(_M_backward_search.current()) > dist;

        // check if no better route can be found
        if (fwd_done && bwd_done) {
            done = true;
        }

        if (!fwd_done) {
            step_forward();
        }
        if (!bwd_done) {
            step_backward();
        }
    }
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
router<Graph, Dijkstra>::distance(const Graph::node_id_type &__node) const {
    if (is_none(__node) || !_M_forward_search.labels().reached(__node) ||
        !_M_backward_search.reached(__node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _M_forward_search.get_label(__node).distance + _M_backward_search.get_label(__node).distance;
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
router<Graph, Dijkstra>::distance() const {
    if (is_none(_M_mid_node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _M_forward_search.get_label(_M_mid_node).distance + _M_backward_search.get_label(_M_mid_node).distance;
}

template<typename Graph, typename Dijkstra>
Graph::path_type
router<Graph, Dijkstra>::route() const {
    if (is_none(_M_mid_node))
        throw std::exception();

    typename Graph::node_id_type fwd_node = _M_mid_node;
    typename Graph::node_id_type bwd_node = _M_mid_node;

    std::deque<typename Graph::node_id_type> p;
    p.push_front(_M_mid_node);

    while (!is_none(fwd_node) && fwd_node != _M_start_node) {
        // if (!_M_graph.topology().has_edge(_M_forward_search.get_label(fwd_node).predecessor, fwd_node
        //  && !_M_graph.topology().has_edge(fwd_node, _M_forward_search.get_label(fwd_node).predecessor)) break;

        fwd_node = _M_forward_search.get_label(fwd_node).predecessor;

        if(is_none(fwd_node)) continue;

        p.push_front(fwd_node);
    }

    while (!is_none(bwd_node) && bwd_node != _M_target_node) {
        // if (!_M_graph.inverse_topology().has_edge(_M_backward_search.get_label(bwd_node).predecessor, bwd_node) || !_M_graph.topology().has_edge(bwd_node, _M_backward_search.get_label(bwd_node).predecessor))
        //     break;

        bwd_node = _M_backward_search.get_label(bwd_node).predecessor;

        if(is_none(bwd_node)) break;

        p.push_back(bwd_node);
    }

    auto result = std::vector<typename Graph::node_id_type>(p.begin(), p.end());
    remove_duplicates_sorted(result);

    return {_M_graph, std::move(result)};
};

template<typename Graph, typename Dijkstra>
Graph::node_id_type
router<Graph, Dijkstra>::mid_node() const {
    return _M_mid_node;
};

template<typename Graph, typename Dijkstra>
bool
router<Graph, Dijkstra>::route_found() const {
    return !is_none(_M_mid_node) && _M_forward_search.reached(_M_mid_node) && _M_backward_search.reached(_M_mid_node) &&
           (_M_forward_search.queue_empty()
            || min_route_distance(_M_forward_search.current()) > distance()) &&
           (_M_backward_search.queue_empty() ||
            min_route_distance(_M_backward_search.current()) > distance());
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::init(Graph::node_id_type __start_node, Graph::node_id_type __target_node) {
    _M_start_node = __start_node;
    _M_target_node = __target_node;
    _M_forward_search.init(_M_start_node, _M_target_node);
    _M_backward_search.init(_M_target_node, _M_start_node);

    _M_mid_node = none_value<typename Graph::node_id_type>;
}
