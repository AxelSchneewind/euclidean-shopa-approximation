#pragma once

#include <cassert>

#include "router.h"

template<typename Graph, typename Dijkstra>
typename router<Graph, Dijkstra>::distance_type
router<Graph, Dijkstra>::min_route_distance(Dijkstra::node_cost_pair_type node) const {
    return node.min_distance();
}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(std::shared_ptr<Graph> graph)
    : _graph{graph},
      _forward_search{_graph},
      _start_node{optional::none_value<typename Graph::node_id_type>},
      _target_node{optional::none_value<typename Graph::node_id_type>},
      _mid_node{optional::none_value<typename Graph::node_id_type>},
      _forward_current{optional::none_value<typename Dijkstra::node_cost_pair_type>} {
}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(router&&routing) noexcept
    : _graph(std::move(routing._graph)),
      _forward_search(std::move(routing._forward_search)),
      _start_node(routing._start_node), _target_node(routing._target_node),
      _mid_node(routing._mid_node),
      _forward_current{routing._forward_current} {
    routing._mid_node = optional::none_value<typename Graph::node_id_type>;
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::step_forward() {
    assert(!_forward_search.queue_empty());

    _forward_search.step();

    // update mid node if target found
    if (_forward_current.node() == _target_node) {
        // TODO why is this not always executed
        _mid_node = _target_node;
    }

    // update current node
    _forward_current = _forward_search.current();
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::compute() {
    assert(!optional::is_none(_start_node));
    assert(optional::is_none(_target_node) || !_forward_search.reached(_target_node));

    bool done = _forward_search.queue_empty();
    while (!done) {
        step_forward();

        done = _forward_search.queue_empty();
        done |= !optional::is_none(_target_node) && _forward_search.reached(_target_node);
        done |= !optional::is_none(_mid_node);
    }

    // TODO make possible to remove
    _mid_node = _target_node;

    assert(optional::is_none(_mid_node) || _mid_node == _target_node);
    assert(optional::is_none(_target_node) || is_infinity(_forward_search.get_label(_target_node).distance()) || (!optional::is_none(_mid_node) && _forward_search.reached(_target_node)));
}

template<typename Graph, typename Dijkstra>
typename router<Graph, Dijkstra>::distance_type
router<Graph, Dijkstra>::distance(const Graph::node_id_type&node) const {
    if (optional::is_none(node) || !_forward_search.reached(node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _forward_search.get_label(node).distance();
}

template<typename Graph, typename Dijkstra>
typename router<Graph, Dijkstra>::distance_type
router<Graph, Dijkstra>::distance() const {
    if (optional::is_none(_mid_node)) {
        return infinity<typename Graph::distance_type>;
    }
    return _forward_search.get_label(_mid_node).distance();
}

template<typename Graph, typename Dijkstra>
typename Graph::path_type
router<Graph, Dijkstra>::route() const {
    if (optional::is_none(_mid_node)) {
        return path<Graph>();
    }

    return _forward_search.path(_mid_node);
};

template<typename Graph, typename Dijkstra>
typename Graph::subgraph_type
router<Graph, Dijkstra>::shortest_path_tree(size_t max_tree_size) const {
    auto tree_fwd = _forward_search.shortest_path_tree(max_tree_size);

    // filter_nodes(tree_fwd, [&](auto node) -> bool {
    //     return _forward_search.get_label(node).distance() + ::distance(_graph.node(node).coordinates, _graph.node(_target_node).coordinates) <= distance();
    // });

    return tree_fwd;
}

template<typename Graph, typename Dijkstra>
bool
router<Graph, Dijkstra>::route_found() const {
    return !optional::is_none(_mid_node) && _forward_search.reached(_mid_node);
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::init(node_id_type start_node, node_id_type target_node) {
    _start_node = start_node;
    _target_node = target_node;
    _mid_node = optional::none_value<node_id_type>;

    _forward_search.init(_start_node, _target_node);

    _forward_current = _forward_search.current();
}
