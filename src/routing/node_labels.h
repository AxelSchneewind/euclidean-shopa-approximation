#pragma once

#include "dijkstra_concepts.h"

#include <vector>

template<RoutableGraph G, typename NodeCostPair>
class node_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;
    using node_cost_pair_type = NodeCostPair;

    const G *d;

    std::vector<node_id_type> _M_touched;
    std::vector<distance_type> _M_distance;
    std::vector<node_id_type> _M_predecessor;

public:
    explicit node_labels(const G *d);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(const node_id_type &__node) const;

    const G::distance_type &distance(const node_id_type &__node) const;

    const node_id_type &predecessor(const node_id_type &__node) const;

    std::span<const node_id_type> all_visited() const;

    void label(const node_cost_pair_type &__node_cost_pair);
};


template<RoutableGraph G, typename N>
std::span<const typename node_labels<G, N>::node_id_type>
node_labels<G, N>::all_visited() const {
    return std::span<const node_id_type>(_M_touched.begin(), _M_touched.end());
}


template<RoutableGraph G, typename N>
node_labels<G, N>::node_labels(const G *d)
        : d(d), _M_predecessor(d->node_count(), NO_NODE_ID),
          _M_distance(d->node_count(), DISTANCE_INF) {
    _M_touched.reserve(std::sqrt(d->node_count()));
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::init(node_labels<G, N>::node_id_type __start_node, node_labels<G, N>::node_id_type __target_node) {
    for (size_t index = 0; index < _M_touched.size(); ++index) {
        node_labels<G, N>::node_id_type node = _M_touched[index];
        _M_predecessor[node] = NO_NODE_ID;
        _M_distance[node] = DISTANCE_INF;
    }

    _M_touched.clear();
}

template<RoutableGraph Graph, typename N>
const node_labels<Graph, N>::node_id_type &
node_labels<Graph, N>::predecessor(const node_labels<Graph, N>::node_id_type &node) const {
    if (node == NO_NODE_ID) return NO_NODE_ID;
    return _M_predecessor[node];
}

template<RoutableGraph G, typename N>
const G::distance_type &
node_labels<G, N>::distance(const node_labels<G, N>::node_id_type &node) const {
    assert(node != NO_NODE_ID);
    return _M_distance[node];
}

template<RoutableGraph G, typename N>
bool
node_labels<G, N>::reached(const node_labels<G, N>::node_id_type &node) const {
    assert(node != NO_NODE_ID);
    return _M_predecessor[node] != NO_NODE_ID;
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::label(const node_labels<G, N>::node_cost_pair_type &node_cost_pair) {
    if (_M_predecessor[node_cost_pair.node] == NO_NODE_ID)
        _M_touched.push_back(node_cost_pair.node);
    _M_distance[node_cost_pair.node] = node_cost_pair.distance;
    _M_predecessor[node_cost_pair.node] = node_cost_pair.predecessor;
}
