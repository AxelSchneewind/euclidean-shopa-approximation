#pragma once

#include "node_labels.h"


template<RoutableGraph G, typename N>
std::span<const typename node_labels<G, N>::node_id_type>
node_labels<G, N>::all_visited() const {
    return std::span<const node_id_type>(_touched.begin(), _touched.end());
}


template<RoutableGraph G, typename N>
node_labels<G, N>::node_labels(std::shared_ptr<G> d)
        : _graph(std::move(d)),
          _labels(_graph->node_count(), optional::none_value<N>),
          _node_labelled(_graph->node_count()) {
}

template<RoutableGraph G, typename N>
void
node_labels<G, N>::init(node_labels<G, N>::node_id_type /*start_node*/, node_labels<G, N>::node_id_type /*target_node*/) {
    for (size_t index = 0; index < _touched.size(); ++index) {
        node_id_type node = _touched[index];
        _labels[node] = optional::none_value<N>;
        _node_labelled[node] = false;
    }

    _touched.clear();
}

template<RoutableGraph G, typename N>
bool
node_labels<G, N>::reached(node_labels<G, N>::node_id_type node) const {
    assert(!optional::is_none(node));
    return _node_labelled[node];
    // return !optional::is_none(_labels[node].predecessor);
}

template<RoutableGraph G, typename Label>
Label &node_labels<G, Label>::operator[](node_id_type node) {
    return _labels[node];
}

template<RoutableGraph G, typename Label>
Label &node_labels<G, Label>::at(node_id_type node) {
    return _labels[node];
}

template<RoutableGraph G, typename Label>
Label const&node_labels<G, Label>::at(node_id_type node) const {
    return _labels[node];
}
