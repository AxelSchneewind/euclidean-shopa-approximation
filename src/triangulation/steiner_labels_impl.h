#pragma once

#include "steiner_labels.h"

template<RoutableGraph G, typename N>
// std::vector<typename steiner_labels<G, N>::node_id_type>
steiner_labels<G, N>::label_iterator_type
steiner_labels<G, N>::all_visited() const {
    // std::vector<typename steiner_labels<G, N>::node_id_type> result;

    // for (auto edge: _M_touched) {
    //     for (auto node: _M_graph.node_ids(edge)) {
    //         if (!is_none(node.edge) && node.steiner_index != -1 && reached(node)) {
    //             result.emplace_back(node);
    //         }
    //     }
    // }

    // return result;

    return label_iterator_type(_M_touched.begin(), _M_touched.end(),
                               std::function<steiner_graph::node_id_iterator_type(
                                       steiner_graph::triangle_edge_id_type)>(
                                       [this](steiner_graph::triangle_edge_id_type edge) -> steiner_graph::node_id_iterator_type {
                                           return _M_graph.node_ids(edge);
                                       }));
}


template<RoutableGraph G, typename N>
steiner_labels<G, N>::steiner_labels(G const &__graph)
        : _M_graph(__graph),
          _M_labels(_M_graph.base_graph().edge_count()) {
    _M_touched.reserve(10000);
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::init(steiner_labels<G, N>::node_id_type  /*__start_node*/,
                           steiner_labels<G, N>::node_id_type  /*__target_node*/) {
    for (size_t index = 0; index < _M_touched.size(); ++index) {
        typename G::triangle_edge_id_type edge = _M_touched[index];
        _M_labels[edge] = nullptr;
    }

    _M_touched.clear();
}

template<RoutableGraph G, typename N>
N
steiner_labels<G, N>::get(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));
    if (!_M_labels[__node.edge]) {
        return none_value<N>;
    }
    return (*_M_labels[__node.edge])[__node.steiner_index];
}

template<RoutableGraph G, typename N>
bool
steiner_labels<G, N>::reached(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));

    return _M_labels[__node.edge] &&
           (*_M_labels[__node.edge])[__node.steiner_index].distance != infinity<distance_t>;
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::label(node_id_type __node, N __label) {
    auto edge_id = __node.edge;
    auto count = _M_graph.steiner_info(__node.edge).node_count;

    // assert(_M_touched.empty() ||
    //        _M_graph.topology().has_edge(__label.predecessor, __node));
    assert(__node.edge >= 0 && __node.edge < _M_graph.topology().edge_count());
    assert(count > 0);
    assert(count > __node.steiner_index);

    // if edge has not been touched yet, set up its distance/predecessor arrays
    if (!_M_labels[edge_id]) {
        _M_touched.push_back(edge_id);
        _M_labels[edge_id] = std::make_unique<std::vector<N>>(count, none_value<N>);
    }

    (*_M_labels[edge_id])[__node.steiner_index] = __label;
}
