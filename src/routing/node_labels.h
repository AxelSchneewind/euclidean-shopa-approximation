#pragma once

#include "dijkstra_concepts.h"

#include <vector>

template<RoutableGraph G, typename Label>
class node_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;

    std::shared_ptr<const G> _M_graph;

    std::vector<node_id_type> _M_touched;

    std::vector<Label> _M_labels;
    std::vector<bool> _M_node_labelled;

public:
    using label_type = Label;

    static constexpr size_t SIZE_PER_NODE = sizeof(Label);
    static constexpr size_t SIZE_PER_EDGE = 0;

    explicit node_labels(std::shared_ptr<const G> d);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(node_id_type __node) const;

    G::distance_type distance(node_id_type __node) const;

    node_id_type predecessor(node_id_type __node) const;

    std::span<const node_id_type> all_visited() const;

    void label(node_id_type __node, Label __label);
};

template<RoutableGraph G, typename NodeCostPair>
node_labels<G, NodeCostPair>::label_type
node_labels<G, NodeCostPair>::get(node_id_type __node) const {
    assert(!is_none(__node));
    return _M_labels[__node];
}


template<RoutableGraph G, typename N>
std::span<const typename node_labels<G, N>::node_id_type>
node_labels<G, N>::all_visited() const {
    return std::span<const node_id_type>(_M_touched.begin(), _M_touched.end());
}


template<RoutableGraph G, typename N>
node_labels<G, N>::node_labels(std::shared_ptr<const G> d)
        : _M_graph(d),
          _M_labels(_M_graph->node_count()),
          _M_node_labelled(_M_graph->node_count()){
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::init(node_labels<G, N>::node_id_type __start_node, node_labels<G, N>::node_id_type __target_node) {
    for (size_t index = 0; index < _M_touched.size(); ++index) {
        node_id_type node = _M_touched[index];
        _M_labels[node] = {infinity<distance_type>(), none_value<node_id_type>()};
    }

    _M_touched.clear();
}

template<RoutableGraph G, typename N>
bool
node_labels<G, N>::reached(node_labels<G, N>::node_id_type node) const {
    assert(node);
    return !is_none(_M_labels[node].predecessor);
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::label(node_id_type __node, node_labels<G, N>::label_type __label) {
    if (!_M_node_labelled[__node]) {
        _M_touched.push_back(__node);
        _M_node_labelled[__node] = true;
    }
    _M_labels[__node] = __label;
}
