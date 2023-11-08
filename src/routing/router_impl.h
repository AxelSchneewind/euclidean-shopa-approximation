#pragma once

#include <cassert>

#include "router.h"

#include "../graph/graph_impl.h"
#include "dijkstra_impl.h"
#include <deque>
#include <map>
#include <ostream>

#include "dijkstra_queues.h"

template<typename Graph, typename Dijkstra>
distance_t
router<Graph, Dijkstra>::min_route_distance(const Graph::node_id_type &__node) const {
    distance_t result = 0;

    // TODO check with CH

    if (Dijkstra::search_symmetric) {
        // if node is labelled in _M_forward_search search, it already has its minimal distance and routes using it must be longer
        if (_M_forward_search.reached(__node)) {
            result += _M_forward_search.labels().distance(__node);
        } else if (!_M_forward_search.queue_empty()) {
            // otherwise, its distance must be at least the one of the current node
            result += _M_forward_search.current().distance;
        }

        if (_M_backward_search.reached(__node)) {
            result += _M_backward_search.labels().distance(__node);
        } else if (!_M_backward_search.queue_empty()) {
            result += _M_backward_search.current().distance;
        }
    } else {
        // if labels are not necessarily optimal, only use distances if the node has been labelled already
        if (_M_forward_search.reached(__node)) {
            result += _M_forward_search.labels().distance(__node);
        }
        if (_M_backward_search.reached(__node)) {
            result += _M_backward_search.labels().distance(__node);
        }

        if (!_M_forward_search.reached(__node) && !_M_backward_search.reached(__node)) {
            result += std::max(_M_forward_search.current().distance, _M_backward_search.current().distance);
        }
    }

    return result;
}

template<typename Graph, typename Dijkstra>
Graph::subgraph
router<Graph, Dijkstra>::shortest_path_tree() const {
    std::vector<typename Graph::node_id_type> nodes;
    std::vector<typename Graph::edge_id_type> edges;

    // add nodes and edges of forward dijkstra
    for (auto node_id: _M_forward_search.labels().all_visited()) {
        nodes.push_back(node_id);
        typename Graph::node_id_type pred = _M_forward_search.labels().predecessor(node_id);

        if (is_none(pred.edge) || pred == node_id) continue;

        typename Graph::edge_id_type edge = _M_graph_ptr->topology().edge_id(pred, node_id);
        edges.push_back(edge);
    }

    // add nodes and edges of backward dijkstra
    for (auto node_id: _M_backward_search.labels().all_visited()) {
        nodes.push_back(node_id);

        typename Graph::node_id_type succ = _M_backward_search.labels().predecessor(node_id);

        if (is_none(succ.edge)) continue;

        typename Graph::edge_id_type edge = _M_graph_ptr->topology().edge_id(node_id, succ);
        edges.push_back(edge);
    }

    remove_duplicates(nodes);
    remove_duplicates(edges);

    return {std::move(nodes), std::move(edges)};
}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(std::shared_ptr<const Graph> __graph)
        : _M_graph_ptr(__graph),
          _M_forward_search(this->_M_graph_ptr, __graph->topology()),
          _M_backward_search(this->_M_graph_ptr, __graph->inverse_topology()),
          _M_start_node(none_value<typename Graph::node_id_type>()),
          _M_target_node(none_value<typename Graph::node_id_type>()),
          _M_mid_node(none_value<typename Graph::node_id_type>()) {}

template<typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router(router &&__routing) noexcept
        : _M_graph_ptr(__routing._M_graph_ptr), _M_forward_search(std::move(__routing._M_forward_search)), _M_backward_search(std::move(__routing._M_backward_search)),
          _M_start_node(__routing._M_start_node), _M_target_node(__routing._M_target_node), _M_mid_node(__routing._M_mid_node) {}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::step_forward() {
    assert (!_M_forward_search.queue_empty());

    typename Dijkstra::node_cost_pair current = _M_forward_search.current();
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

    typename Dijkstra::node_cost_pair current = _M_backward_search.current();

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
    // check args
    // if (_M_start_node >= graph->node_count () || _M_target_node >= graph->node_count () || _M_start_node < 0
    //     || _M_target_node < 0 || _M_start_node == NO_NODE_ID || _M_target_node == NO_NODE_ID)
    //   throw;

    bool done = false;
    while (!done) {
        auto dist = distance();
        auto min_fwd = min_route_distance(_M_forward_search.current().node);
        auto min_bwd = min_route_distance(_M_backward_search.current().node);
        bool const fwd_done = _M_forward_search.queue_empty() || min_fwd > dist;
        bool const bwd_done = _M_backward_search.queue_empty() || min_bwd > dist;

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

    // TODO: check if necessary
    if (!_M_forward_search.queue_empty()) {
        step_forward();
    }
    if (!_M_backward_search.queue_empty()) {
        step_backward();
    }
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
router<Graph, Dijkstra>::distance(const Graph::node_id_type &__node) const {
    if (is_none(__node) || !_M_forward_search.labels().reached(__node) || !_M_backward_search.labels().reached(__node)) {
        return infinity<typename Graph::distance_type>();
    }
    return _M_forward_search.labels().distance(__node) + _M_backward_search.labels().distance(__node);
}

template<typename Graph, typename Dijkstra>
Graph::distance_type
router<Graph, Dijkstra>::distance() const {
    if (is_none(_M_mid_node)) {
        return infinity<typename Graph::distance_type>();
    }
    return _M_forward_search.labels().distance(_M_mid_node) + _M_backward_search.labels().distance(_M_mid_node);
}

template<typename Graph, typename Dijkstra>
Graph::path
router<Graph, Dijkstra>::route() const {
    if (is_none(_M_mid_node))
        throw std::exception();

    typename Graph::node_id_type fwd_node = _M_mid_node;
    typename Graph::node_id_type bwd_node = _M_mid_node;

    std::deque<typename Graph::node_id_type> p;
    p.push_front(_M_mid_node);

    while (!is_none(fwd_node) && fwd_node != _M_start_node) {
        assert (_M_graph_ptr->topology().has_edge(_M_forward_search.labels().predecessor(fwd_node), fwd_node));

        fwd_node = _M_forward_search.labels().predecessor(fwd_node);

        assert(!is_none(fwd_node));

        p.push_front(fwd_node);
    }

    while (!is_none(bwd_node) && bwd_node != _M_target_node) {
        assert (_M_graph_ptr->inverse_topology().has_edge(_M_backward_search.labels().predecessor(bwd_node), bwd_node));
        assert (_M_graph_ptr->topology().has_edge(bwd_node, _M_backward_search.labels().predecessor(bwd_node)));

        bwd_node = _M_backward_search.labels().predecessor(bwd_node);

        assert(!is_none(bwd_node));

        p.push_back(bwd_node);
    }

    auto result = std::vector<typename Graph::node_id_type>(p.begin(), p.end());
    remove_duplicates_sorted(result);

    return { result };
};

template<typename Graph, typename Dijkstra>
Graph::node_id_type
router<Graph, Dijkstra>::mid_node() const {
    return _M_mid_node;
};

template<typename Graph, typename Dijkstra>
bool
router<Graph, Dijkstra>::route_found() const {
    return !is_none(_M_mid_node) && _M_forward_search.reached(_M_mid_node) && _M_backward_search.reached(_M_mid_node);
}

template<typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::init(Graph::node_id_type __start_node, Graph::node_id_type __target_node) {
    // check args
    //if (__start_node >= graph->node_count () || __target_node >= graph->node_count () || __start_node < 0
    //  || __target_node < 0 || __start_node == NO_NODE_ID || __target_node == NO_NODE_ID)
    // throw;

    _M_start_node = __start_node;
    _M_target_node = __target_node;
    _M_forward_search.init(_M_start_node, _M_target_node);
    _M_backward_search.init(_M_target_node, _M_start_node);

    _M_mid_node = none_value<typename Graph::node_id_type>();
}
