#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/bidirectional_adjacency_list.h"
#include "../graph/graph.h"
#include "dijkstra_rules.h"

#include <queue>
#include <vector>
using std::vector;

#include <cmath>
#include <concepts>
#include <iostream>


template<typename Info>
struct node_cost_pair_t {
    node_id_t node;
    Info info;
    distance_t distance;
    node_id_t predecessor;
};

template<typename N, typename E>
class dijkstra;

template<typename N, typename E>
class dijkstra {
public:
    using node_cost_pair = node_cost_pair_t<std::tuple<>>;
    struct Comp {
        dijkstra<N,E>* d;
        bool operator()(const node_cost_pair& n1, const node_cost_pair& n2) { return cost_ascending(n1, n2, *d); };
    };
    struct UseEdge {
        dijkstra<N,E>* d;
        bool operator()(const node_id_t& node, const edge_id_t& via) {
            return true;
        };
    };

private:
    const bidirectional_adjacency_list<E> graph;

    node_id_t start_node;
    node_id_t target_node;

    std::priority_queue<node_cost_pair, vector<node_cost_pair>, Comp> queue;
    UseEdge useEdge;

    vector<node_id_t> _visited;
    vector<distance_t> _distance;
    vector<node_id_t> _predecessor;

    // step without pulling from queue but using the given node instead
    void step(const node_cost_pair &node);

public:
    // constructs a dijkstra object for the given graph
    explicit dijkstra(const bidirectional_adjacency_list<E> &graph);
    dijkstra(dijkstra &&dijkstra) noexcept;

    ~dijkstra() = default;

    const distance_t &distance(const node_id_t &node) const;
    const node_id_t &predecessor(const node_id_t &node) const;

    // init one to one
    void init(node_id_t start_node, node_id_t target_node);
    // init one to all
    void init(node_id_t start_node);

    // get the current node without removing from queue
    const node_cost_pair &current() const;

    // get the current nodes without removing from queue
    std::vector<node_cost_pair> current_nodes();

    // step with pulling from queue
    void step();

    // TODO: remove
    // label node with distance and predecessor
    void label(const node_id_t &node, const distance_t &distance, const node_id_t &predecessor);
    void label(const node_cost_pair &node_cost_pair);
    // label with pulling from queue
    void label();

    // push node into queue
    void push(const node_id_t &node, const distance_t &distance, const node_id_t &predecessor);

    // function to check whether search is finished (queue empty)
    bool done() const;

    // check if node is reached (labelled)
    bool reached(const node_id_t &node) const;
    bool reachedTarget() const;
};





template<typename N, typename E>
const node_id_t &
dijkstra<N, E>::predecessor(const node_id_t &node) const { return _predecessor[node]; }
template<typename N, typename E>
const distance_t &
dijkstra<N, E>::distance(const node_id_t &node) const { return _distance[node]; }

template<typename N, typename E>
dijkstra<N, E>::dijkstra(const bidirectional_adjacency_list<E> &graph)
    : graph(graph),
      _distance(graph.node_count(), DISTANCE_INF),
      _predecessor(graph.node_count(), NO_NODE_ID),
      useEdge(UseEdge{this}),
      queue(Comp{this}) {
    _visited.reserve(std::ceil(std::sqrt(graph.node_count() / 100)));
    init(NO_NODE_ID);
}

template<typename N, typename E>
dijkstra<N, E>::dijkstra(dijkstra<N, E> &&other) noexcept
    : graph(other.graph),
      _distance(std::move(other._distance)),
      _predecessor(std::move(other._predecessor)),
      _visited(std::move(other._visited)),
      queue(std::move(other.queue)) {
}

template<typename N, typename E>
const dijkstra<N, E>::node_cost_pair &
dijkstra<N, E>::current() const {
    return queue.top();
}

template<typename N, typename E>
std::vector<typename dijkstra<N, E>::node_cost_pair>
dijkstra<N, E>::current_nodes() {
    std::vector<dijkstra<N, E>::node_cost_pair> result;
    result.reserve(queue.size());
    while (!queue.empty()) {
        result.emplace_back(queue.top());
        queue.pop();
    }

    for (auto element: result)
        queue.push(element);
    return result;
}

template<typename N, typename E>
bool
dijkstra<N, E>::reached(const node_id_t &node) const {
    return node != NO_NODE_ID && _distance[node] != DISTANCE_INF;
}

template<typename N, typename E>
bool
dijkstra<N, E>::reachedTarget() const {
    return target_node == NO_NODE_ID ? done() : reached(target_node);
}

template<typename N, typename E>
bool
dijkstra<N, E>::done() const { return queue.empty(); }

template<typename N, typename E>
void
dijkstra<N, E>::push(const node_id_t &node, const distance_t &distance, const node_id_t &predecessor) {
    dijkstra<N, E>::node_cost_pair p;
    p.node = node;
    p.distance = distance;
    p.predecessor = predecessor;
    queue.push(p);
}

template<typename N, typename E>
void
dijkstra<N, E>::label(const node_id_t &node, const distance_t &distance, const node_id_t &predecessor) {
    _distance[node] = distance;
    _predecessor[node] = predecessor;
    _visited.push_back(node);
}
template<typename N, typename E>
void
dijkstra<N, E>::label(const dijkstra<N, E>::node_cost_pair &node_cost_pair) {
    _distance[node_cost_pair.node] = node_cost_pair.distance;
    _predecessor[node_cost_pair.node] = node_cost_pair.predecessor;
    _visited.push_back(node_cost_pair.node);
}

template<typename N, typename E>
void
dijkstra<N, E>::init(node_id_t start_node, node_id_t target_node) {
    this->target_node = target_node;

    if (start_node != NO_NODE_ID && start_node == this->start_node)
        return;
    this->start_node = start_node;

    // label all visited nodes with default values
    for (node_id_t i = 0; i < _visited.size(); i++) {
        label(_visited[i], DISTANCE_INF, NO_NODE_ID);
        _visited.pop_back();
    }

    _visited.clear();
    while (!queue.empty())
        queue.pop();

    if (start_node != NO_NODE_ID) push(start_node, 0, start_node);
}
template<typename N, typename E>
void
dijkstra<N, E>::init(node_id_t start_node) {
    init(start_node, NO_NODE_ID);
}

template<typename N, typename E>
void
dijkstra<N, E>::step(const dijkstra<N, E>::node_cost_pair &node) {
    auto edges = graph.forward().node_edges(node.node);
    for (const adjacency_list_edge_t<E> &edge: edges) {
        // ignore certain edges
        //if (!useEdge(edgeIndex++))
        //continue;

        const node_id_t &successor = edge.destination;
        const distance_t &successorCost = _distance[successor];
        const distance_t newCost = node.distance + edge.info.cost;

        if (newCost < successorCost)
            // (re-)insert node into the queue with updated priority
            push(successor, newCost, node.node);
    }
}
template<typename N, typename E>
void
dijkstra<N, E>::step() {
    // remove already settled nodes
    while (!queue.empty() && current().distance >= distance(current().node))
        queue.pop();

    if (queue.empty())
        return;

    node_cost_pair_t node = current();
    step(node);

    // label current node
    label(node.node, node.distance, node.predecessor);

    // remove current node
    queue.pop();
}


template<typename N, typename E>
void
dijkstra<N, E>::label() {
    if (queue.empty())
        throw;

    node_cost_pair_t node = current();

    // label current node
    label(node.node, node.distance, node.predecessor);

    // remove current node
    queue.pop();

    // remove already settled nodes
    while (!queue.empty() && current().distance >= _distance[current().node])
        queue.pop();
}
