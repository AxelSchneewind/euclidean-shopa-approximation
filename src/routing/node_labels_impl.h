#pragma once

#include "node_labels.h"


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
node_labels<G, N>::node_labels(G const &d)
        : _M_graph(d),
          _M_labels(_M_graph.node_count(), none_value<N>),
          _M_node_labelled(_M_graph.node_count()) {
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::init(node_labels<G, N>::node_id_type __start_node, node_labels<G, N>::node_id_type __target_node) {
    for (size_t index = 0; index < _M_touched.size(); ++index) {
        node_id_type node = _M_touched[index];
        _M_labels[node] = none_value<N>;
    }

    _M_touched.clear();
}

template<RoutableGraph G, typename N>
bool
node_labels<G, N>::reached(node_labels<G, N>::node_id_type node) const {
    assert(!is_none(node));
    return _M_node_labelled[node];
    // return !is_none(_M_labels[node].predecessor);
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
