#pragma once

#include <cassert>

#include "../graph/graph.h"
#include "dijkstra.h"
#include <deque>
#include <map>
#include <ostream>



template<typename N, typename E>
class routing_t {
protected:
    std::shared_ptr<const graph_t<N, E>> graph;

    dijkstra<N, E> forward;
    dijkstra<N, E> backward;

    node_id_t start_node;
    node_id_t target_node;

    // node where forward and backward search met
    node_id_t _mid_node;

    void step_forward();
    void step_backward();

public:
    explicit routing_t(const graph_t<N, E> *graph);
    routing_t(routing_t &&other) noexcept ;

    // calculates a one to one route using bidirectional dijkstra
    // TODO: when searches meet, iterate through queues (without pulling) and check for shorter paths
    void compute_route(node_id_t start_node, node_id_t target_node);

    // returns the distance of the calculated route
    distance_t distance() const;
    // returns the fistance of the given node (or infinity if it has not been found by both searches)
    distance_t distance(const node_id_t &node) const;

    // returns the nodes of the path from start to target node
    graph_t<N,E>::path_t route() const;

    // returns the node where forward and backward search met
    node_id_t mid_node() const;
};








template<typename N, typename E>
routing_t<N, E>::routing_t(const graph_t<N, E> *graph) : graph(graph),
                                                         forward(graph->forward()),
                                                         backward(graph->backward()),
                                                         start_node(NO_NODE_ID),
                                                         target_node(NO_NODE_ID),
                                                         _mid_node(NO_NODE_ID) {
}

template<typename N, typename E>
routing_t<N, E>::routing_t(routing_t &&routing) noexcept : graph(routing.graph),
                                                           forward(std::move(routing.forward)),
                                                           backward(std::move(routing.backward)),
                                                           start_node(routing.start_node),
                                                           target_node(routing.target_node),
                                                           _mid_node(routing._mid_node) {
}

template<typename N, typename E>
void
routing_t<N, E>::step_forward() {
    assert(!forward.done());

    node_cost_pair_t current = forward.current();

    forward.step();

    // check if searches met and provide best result yet
    if (backward.reached(current.node) && distance(current.node) < distance(mid_node()))
        _mid_node = current.node;
}

template<typename N, typename E>
void
routing_t<N, E>::step_backward() {
    assert(!backward.done());

    node_cost_pair_t current = backward.current();

    backward.step();

    // check if searches met and provide best result yet
    if (forward.reached(current.node) && distance(current.node) < distance(mid_node()))
        _mid_node = current.node;
}

template<typename N, typename E>
void
routing_t<N, E>::compute_route(node_id_t start_node, node_id_t target_node) {
    // check args
    if (start_node >= graph->node_count() || target_node >= graph->node_count() || start_node < 0 || target_node < 0 || start_node == NO_NODE_ID || target_node == NO_NODE_ID)
        throw;

    forward.init(start_node, target_node);
    backward.init(graph->backward_node_id(target_node), graph->backward_node_id(start_node));

    _mid_node = NO_NODE_ID;

    while ((!forward.done() || !backward.done())) {
        bool fwd_done = forward.done();
        bool bwd_done = backward.done();

        if (fwd_done && bwd_done) break;

        if (!fwd_done)
            step_forward();
        if (!bwd_done)
            step_backward();
    }

    // label all enqueued nodes
    //for (auto &ncp: forward.current_nodes())
        //forward.label(ncp);
    //for (auto &ncp: backward.current_nodes())
        //backward.label(ncp);

    // search enqueued nodes for better mid_node
    for (auto &ncp: forward.current_nodes())
        if (backward.reached(ncp.node) && distance(ncp.node) < distance(mid_node()))
            _mid_node = ncp.node;
    for (auto &ncp: backward.current_nodes())
        if (forward.reached(ncp.node) && distance(ncp.node) < distance(mid_node()))
            _mid_node = ncp.node;
}

template<typename N, typename E>
distance_t
routing_t<N, E>::distance(const node_id_t &node) const {
    if (node == NO_NODE_ID || !forward.reached(node) || !backward.reached(node))
        return DISTANCE_INF;
    return forward.distance(node) + backward.distance(node);
}

template<typename N, typename E>
distance_t
routing_t<N, E>::distance() const {
    if (_mid_node == NO_NODE_ID)
        return DISTANCE_INF;
    return forward.distance(_mid_node) + backward.distance(_mid_node);
}

template<typename N, typename E>
graph_t<N,E>::path_t
routing_t<N, E>::route() const {
    std::deque<node_id_t> p;

    if (_mid_node == NO_NODE_ID)
        return typename graph_t<N,E>::path_t(std::vector(p.begin(), p.end()));

    node_id_t fwd_node = _mid_node;
    node_id_t bwd_node = _mid_node;

    p.push_front(_mid_node);

    // TODO unpack shortcuts
    while (fwd_node != NO_NODE_ID && fwd_node != start_node) {
        fwd_node = forward.predecessor(fwd_node);
        p.push_front(fwd_node);
    }

    while (bwd_node != NO_NODE_ID && bwd_node != target_node) {
        bwd_node = graph->forward_node_id(backward.predecessor(bwd_node));
        p.push_back(bwd_node);
    }

    typename graph_t<N,E>::path_t result{ std::vector<node_id_t>(p.begin(), p.end()) };
    return result;
};


template<typename N, typename E>
node_id_t
routing_t<N, E>::mid_node() const { return _mid_node; };


template <typename N, typename E>
std::ostream &
operator<<(std::ostream &stream, typename graph_t<N,E>::path_t &r) {
    stream << "{ ";

    int length = r.nodes.size();
    if (length > 0) {
        for (int i = 0; i < length - 1; i++)
            stream << r.nodes[i] << ", ";
        stream << r.nodes[length - 1];
    }

    return stream << " }" << std::flush;
}
