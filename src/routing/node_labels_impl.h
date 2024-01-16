#pragma once

#include "node_labels.h"


template<RoutableGraph G, typename NodeCostPair>
node_labels<G, NodeCostPair>::label_type
node_labels<G, NodeCostPair>::get(node_id_type node) const {
    assert(!is_none(node));
    return _labels[node];
}


template<RoutableGraph G, typename N>
std::span<const typename node_labels<G, N>::node_id_type>
node_labels<G, N>::all_visited() const {
    return std::span<const node_id_type>(_touched.begin(), _touched.end());
}


template<RoutableGraph G, typename N>
node_labels<G, N>::node_labels(G const &d)
        : _graph(d),
          _labels(_graph.node_count(), none_value<N>),
          _node_labelled(_graph.node_count()) {
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::init(node_labels<G, N>::node_id_type start_node, node_labels<G, N>::node_id_type target_node) {
    for (size_t index = 0; index < _touched.size(); ++index) {
        node_id_type node = _touched[index];
        _labels[node] = none_value<N>;
        _node_labelled[node] = false;
    }

    _touched.clear();
}

template<RoutableGraph G, typename N>
bool
node_labels<G, N>::reached(node_labels<G, N>::node_id_type node) const {
    assert(!is_none(node));
    return _node_labelled[node];
    // return !is_none(_labels[node].predecessor);
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::label(node_id_type const& node, N const& label) {
    if (!_node_labelled[node]) {
        _touched.emplace_back(node);
        _node_labelled[node] = true;
    }
    _labels[node] = label;
}
