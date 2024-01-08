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
router<Graph, Dijkstra>::min_route_distance(Dijkstra::node_cost_pair_type node) const {
    return node.min_distance();
}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(Graph const &graph)
        : _M_graph(graph),
          _M_forward_search(_M_graph),
          _M_start_node(none_value<typename Graph::node_id_type>),
          _M_target_node(none_value<typename Graph::node_id_type>),
          _M_mid_node(none_value<typename Graph::node_id_type>) {}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(router &&routing) noexcept
        : _M_graph(routing._M_graph),
          _M_forward_search(std::move(routing._M_forward_search)),
          _M_start_node(routing._M_start_node), _M_target_node(routing._M_target_node),
          _M_mid_node(routing._M_mid_node) {
    routing._M_mid_node = none_value<typename Graph::node_id_type>;
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::step_forward() {
    assert (!_M_forward_search.queue_empty());

    _forward_current = _M_forward_search.current();
    assert(!is_none(_forward_current.node));

    _M_forward_search.step();

    if (_forward_current.node == _M_target_node)
        _M_mid_node = _M_target_node;
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::compute_route() {
    bool done = _M_forward_search.queue_empty() || min_route_distance(_M_forward_search.current()) > distance();
    std::size_t step_count = 0;
    while (!done) {
        step_forward();

        done = _M_forward_search.queue_empty() || min_route_distance(_M_forward_search.current()) > distance();
        step_count++;
    }
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
router<Graph, Dijkstra>::distance(const Graph::node_id_type &node) const {
    if (is_none(node) || !_M_forward_search.reached(node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _M_forward_search.get_label(node).distance;
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
router<Graph, Dijkstra>::distance() const {
    if (is_none(_M_mid_node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _M_forward_search.get_label(_M_mid_node).distance;
}

template<typename Graph, typename Dijkstra>
Graph::path_type
router<Graph, Dijkstra>::route() const {
    if (is_none(_M_mid_node))
        return path<Graph>();

    auto path_fwd = _M_forward_search.path(_M_mid_node);
    return path_fwd;
};

template<typename Graph, typename Dijkstra>
Graph::subgraph_type
router<Graph, Dijkstra>::shortest_path_tree() const {
    auto tree_fwd = _M_forward_search.shortest_path_tree();

    // filter_nodes(tree_fwd, [&](auto node) -> bool {
    //     return _M_forward_search.get_label(node).distance + ::distance(_M_graph.node(node).coordinates, _M_graph.node(_M_target_node).coordinates) <= distance();
    // });

    return tree_fwd;
}

template<typename Graph, typename Dijkstra>
bool
router<Graph, Dijkstra>::route_found() const {
    return !is_none(_M_mid_node) && _M_forward_search.reached(_M_mid_node);
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::init(Graph::node_id_type start_node, Graph::node_id_type target_node) {
    _M_start_node = start_node;
    _M_target_node = target_node;

    _M_forward_search.init(_M_start_node, _M_target_node);

    _forward_current = _M_forward_search.current();
}
