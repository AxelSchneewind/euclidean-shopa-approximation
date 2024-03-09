#pragma once

#include <cassert>

#include "router_bidirectional.h"


template<typename Graph, typename Dijkstra>
typename bidirectional_router<Graph, Dijkstra>::distance_type
bidirectional_router<Graph, Dijkstra>::min_route_distance(node_cost_pair_type node) const {
    return node.value();
}

template<typename Graph, typename Dijkstra>
bidirectional_router<Graph, Dijkstra>::bidirectional_router(std::shared_ptr<Graph> graph)
    : base(graph),
      _backward_search(base::_graph) {
}

template<typename Graph, typename Dijkstra>
bidirectional_router<Graph, Dijkstra>::bidirectional_router(bidirectional_router&&routing) noexcept
        : base(routing._graph),
          _backward_search(std::move(routing._backward_search)) {
    routing._mid_node = optional::none_value<typename Graph::node_id_type>;
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::step_forward() {
    assert(!base::_forward_search.queue_empty());

    base::_forward_current = base::_forward_search.current();
    assert(!optional::is_none(base::_forward_current.node()));

    base::_forward_search.step();

    // check if searches met and provide best result so far
    if (_backward_search.reached(base::_forward_current.node()) && distance(base::_forward_current.node()) < distance()) {
        base::_mid_node = base::_forward_current.node();
    }
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::step_backward() {
    assert(!_backward_search.queue_empty());

    _backward_current = _backward_search.current();
    assert(!optional::is_none(_backward_current.node()));

    _backward_search.step();

    // check if searches met and provide best result yet
    if (base::_forward_search.reached(_backward_current.node()) && distance(_backward_current.node()) < distance()) {
        base::_mid_node = _backward_current.node();
    }
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::compute() {
    bool done = false;
    std::size_t step_count = 0;
    while (!done) {
        auto dist = distance();
        bool const fwd_done = base::_forward_search.queue_empty() || min_route_distance(base::_forward_search.current()) > dist;
        bool const bwd_done = _backward_search.queue_empty() || min_route_distance(_backward_search.current()) > dist;

        // check if no better route can be found
        done |= fwd_done && bwd_done;

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
typename bidirectional_router<Graph, Dijkstra>::distance_type
bidirectional_router<Graph, Dijkstra>::distance(const node_id_type& node) const {
    if (optional::is_none(node) || !base::_forward_search.reached(node) ||
        !_backward_search.reached(node)) {
        return infinity<typename Graph::distance_type>;
    }
    return base::_forward_search.get_label(node).distance() + _backward_search.get_label(node).distance();
}

template<typename Graph, typename Dijkstra>
typename bidirectional_router<Graph, Dijkstra>::distance_type
bidirectional_router<Graph, Dijkstra>::distance() const {
    if (optional::is_none(base::_mid_node)) {
        return infinity<typename Graph::distance_type>;
    }
    return base::_forward_search.get_label(base::_mid_node).distance() + _backward_search.get_label(base::_mid_node).distance();
}

template<typename Graph, typename Dijkstra>
Graph::path_type
bidirectional_router<Graph, Dijkstra>::route() const {
    if (optional::is_none(base::_mid_node))
        return path<Graph>();

    auto path_fwd = base::_forward_search.path(base::_mid_node);
    auto path_bwd = _backward_search.path(_backward_search.get_label(base::_mid_node).predecessor());
    path_bwd.invert();

    return path<Graph>::concat(path_fwd, path_bwd);
};

template<typename Graph, typename Dijkstra>
Graph::subgraph_type
bidirectional_router<Graph, Dijkstra>::shortest_path_tree(size_t max_tree_size) const {
    auto tree_fwd = base::_forward_search.shortest_path_tree(max_tree_size / 2);
    auto tree_bwd = _backward_search.shortest_path_tree(max_tree_size / 2);

    // filter_nodes(tree_fwd, [&](auto node) -> bool {
    //     return _forward_search.get_label(node).distance() + ::distance(_graph.node(node).coordinates, _graph.node(_target_node).coordinates) <= distance();
    // });
    // filter_nodes(tree_bwd, [&](auto node) -> bool {
    //     return _backward_search.get_label(node).distance() + ::distance(_graph.node(node).coordinates, _graph.node(_start_node).coordinates) <= distance();
    // });

    return subgraphs_union<Graph>(tree_fwd, tree_bwd);
}

template<typename Graph, typename Dijkstra>
typename bidirectional_router<Graph, Dijkstra>::node_id_type
bidirectional_router<Graph, Dijkstra>::mid_node() const {
    return base::_mid_node;
};

template<typename Graph, typename Dijkstra>
bool
bidirectional_router<Graph, Dijkstra>::route_found() const {
    return !optional::is_none(base::_mid_node) && base::_forward_search.reached(base::_mid_node) && _backward_search.reached(base::_mid_node);
}

template<typename Graph, typename Dijkstra>
void
bidirectional_router<Graph, Dijkstra>::init(node_id_type start_node, node_id_type target_node) {
    base::_start_node = start_node;
    base::_target_node = target_node;
    base::_mid_node = optional::none_value<node_id_type>;

    base::_forward_search.init(base::_start_node, base::_target_node);
    _backward_search.init(base::_target_node, base::_start_node);

    base::_forward_current = base::_forward_search.current();
    _backward_current = _backward_search.current();
}
