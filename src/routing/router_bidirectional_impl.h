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
bidirectional_router<Graph, Dijkstra>::min_route_distance(Dijkstra::node_cost_pair_type node) const {
    return node.min_distance();
}

template<typename Graph, typename Dijkstra>
bidirectional_router<Graph, Dijkstra>::bidirectional_router(Graph const &graph)
        : _M_graph(graph),
          _M_forward_search(_M_graph),
          _M_backward_search(_M_graph),
          _M_start_node(none_value<typename Graph::node_id_type>),
          _M_target_node(none_value<typename Graph::node_id_type>),
          _M_mid_node(none_value<typename Graph::node_id_type>) {}

template<typename Graph, typename Dijkstra>
bidirectional_router<Graph, Dijkstra>::bidirectional_router(bidirectional_router &&routing) noexcept
        : _M_graph(routing._M_graph),
          _M_forward_search(std::move(routing._M_forward_search)),
          _M_backward_search(std::move(routing._M_backward_search)),
          _M_start_node(routing._M_start_node), _M_target_node(routing._M_target_node),
          _M_mid_node(routing._M_mid_node) {
    routing._M_mid_node = none_value<typename Graph::node_id_type>;
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::step_forward() {
    assert (!_M_forward_search.queue_empty());

    _forward_current = _M_forward_search.current();
    assert(!is_none(_forward_current.node));

    _M_forward_search.step();

    // check if searches met and provide best result so far
    if (_M_backward_search.reached(_forward_current.node) && distance(_forward_current.node) < distance()) {
        _M_mid_node = _forward_current.node;
    }
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::step_backward() {
    assert (!_M_backward_search.queue_empty());

    _backward_current = _M_backward_search.current();
    assert(!is_none(_backward_current.node));

    _M_backward_search.step();

    // check if searches met and provide best result yet
    if (_M_forward_search.reached(_backward_current.node) && distance(_backward_current.node) < distance()) {
        _M_mid_node = _backward_current.node;
    }
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::compute_route() {
    bool done = false;
    std::size_t step_count = 0;
    while (!done) {
        auto dist = std::min(distance(), 2 * ::distance(_M_graph.node(_M_start_node).coordinates, _M_graph.node(_M_target_node).coordinates));
        bool const fwd_done =
                _M_forward_search.queue_empty() || min_route_distance(_M_forward_search.current()) > dist;
        bool const bwd_done = _M_backward_search.reached(_M_target_node);
                // _M_backward_search.queue_empty() || min_route_distance(_M_backward_search.current()) > dist;

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

        step_count++;
    }
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
bidirectional_router<Graph, Dijkstra>::distance(const Graph::node_id_type &node) const {
    if (is_none(node) || !_M_forward_search.reached(node) ||
        !_M_backward_search.reached(node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _M_forward_search.get_label(node).distance + _M_backward_search.get_label(node).distance;
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
bidirectional_router<Graph, Dijkstra>::distance() const {
    if (is_none(_M_mid_node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _M_forward_search.get_label(_M_mid_node).distance + _M_backward_search.get_label(_M_mid_node).distance;
}

template<typename Graph, typename Dijkstra>
Graph::path_type
bidirectional_router<Graph, Dijkstra>::route() const {
    if (is_none(_M_mid_node))
        return path<Graph>();

    auto path_fwd = _M_forward_search.path(_M_mid_node);
    auto path_bwd = _M_backward_search.path(_M_backward_search.get_label(_M_mid_node).predecessor);
    path_bwd.invert();

    return path<Graph>::concat(path_fwd, path_bwd);
};

template<typename Graph, typename Dijkstra>
Graph::subgraph_type
bidirectional_router<Graph, Dijkstra>::shortest_path_tree() const {
    auto tree_fwd = _M_forward_search.shortest_path_tree();
    auto tree_bwd = _M_backward_search.shortest_path_tree();

    // filter_nodes(tree_fwd, [&](auto node) -> bool {
    //     return _M_forward_search.get_label(node).distance + ::distance(_M_graph.node(node).coordinates, _M_graph.node(_M_target_node).coordinates) <= distance();
    // });
    // filter_nodes(tree_bwd, [&](auto node) -> bool {
    //     return _M_backward_search.get_label(node).distance + ::distance(_M_graph.node(node).coordinates, _M_graph.node(_M_start_node).coordinates) <= distance();
    // });

    return subgraphs_union<Graph>(tree_fwd, tree_bwd);
}

template<typename Graph, typename Dijkstra>
Graph::node_id_type
bidirectional_router<Graph, Dijkstra>::mid_node() const {
    return _M_mid_node;
};

template<typename Graph, typename Dijkstra>
bool
bidirectional_router<Graph, Dijkstra>::route_found() const {
    return !is_none(_M_mid_node) && _M_forward_search.reached(_M_mid_node) && _M_backward_search.reached(_M_mid_node);
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::init(Graph::node_id_type start_node, Graph::node_id_type target_node) {
    _M_start_node = start_node;
    _M_target_node = target_node;

    _M_forward_search.init(_M_start_node, _M_target_node);
    _M_backward_search.init(_M_target_node, _M_start_node);

    _forward_current = _M_forward_search.current();
    _backward_current = _M_backward_search.current();
}
