#pragma once

#include "dijkstra_concepts.h"

#include <vector>

template<RoutableGraph G, typename NodeCostPair>
class node_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;
    using node_cost_pair_type = NodeCostPair;
    struct label { distance_type distance; node_id_type predecessor; };

    std::shared_ptr<const G> _M_graph;

    std::vector<node_id_type> _M_touched;
    std::vector<label> _M_labels;

public:
    explicit node_labels(std::shared_ptr<const G> d);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(node_id_type __node) const;

    G::distance_type distance(node_id_type __node) const;

    node_id_type predecessor(node_id_type __node) const;

    std::span<const node_id_type> all_visited() const;

    void label(node_cost_pair_type __node_cost_pair);
};


template<RoutableGraph G, typename N>
std::span<const typename node_labels<G, N>::node_id_type>
node_labels<G, N>::all_visited() const {
    return std::span<const node_id_type>(_M_touched.begin(), _M_touched.end());
}


template<RoutableGraph G, typename N>
node_labels<G, N>::node_labels(std::shared_ptr<const G> d)
        : _M_graph(d),
          _M_labels(_M_graph->node_count(), { infinity<distance_type>(), none_value<node_id_type>() }) {
    _M_touched.reserve(10000);
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::init(node_labels<G, N>::node_id_type __start_node, node_labels<G, N>::node_id_type __target_node) {
    for (size_t index = 0; index < _M_touched.size(); ++index) {
        node_id_type node = _M_touched[index];
        _M_labels[node] = { infinity<distance_type>(), none_value<node_id_type>() };
    }

    _M_touched.clear();
}

template<RoutableGraph Graph, typename N>
node_labels<Graph, N>::node_id_type
node_labels<Graph, N>::predecessor(node_labels<Graph, N>::node_id_type node) const {
    assert(!is_none(node));
    return _M_labels[node].predecessor;
}

template<RoutableGraph G, typename N>
G::distance_type
node_labels<G, N>::distance(node_labels<G, N>::node_id_type node) const {
    assert(!is_none(node));
    return _M_labels[node].distance;
}

template<RoutableGraph G, typename N>
bool
node_labels<G, N>::reached(node_labels<G, N>::node_id_type node) const {
    assert(node);
    return !is_none(_M_labels[node].predecessor);
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::label(node_labels<G, N>::node_cost_pair_type node_cost_pair) {
    if (is_none(_M_labels[node_cost_pair.node].predecessor))
        _M_touched.push_back(node_cost_pair.node);
    _M_labels[node_cost_pair.node] = { node_cost_pair.distance, node_cost_pair.predecessor};
}
