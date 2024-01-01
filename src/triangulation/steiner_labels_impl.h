#pragma once

#include "steiner_labels.h"
#include "node_info_container.h"

template<RoutableGraph G, typename N>
steiner_labels<G, N>::label_iterator_type
steiner_labels<G, N>::all_visited() const {
    return label_iterator_type(_M_edge_touched.begin(), _M_edge_touched.end(),
                               std::function<steiner_graph::node_id_iterator_type(std::size_t)>(
                                       [this](std::size_t edge) -> steiner_graph::node_id_iterator_type {
                                           return _M_graph.node_ids(edge);
                                       }));
}

template<RoutableGraph G, typename N>
steiner_labels<G, N>::steiner_labels(G const &graph)
        : _M_graph(graph),
        //_M_labels(_M_graph.subdivision_info().offsets(), 0, none_value<N>),
          _M_labels(_M_graph.subdivision_info().offsets(), none_value<N>),
          _M_base_labels(_M_graph.base_graph().node_count(), none_value<N>),
          _M_edge_touched(_M_graph.base_graph().edge_count(), false){
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::init(steiner_labels<G, N>::node_id_type  /*__start_node*/,
                           steiner_labels<G, N>::node_id_type  /*__target_node*/) {
    for (size_t index = 0; index < _M_edge_touched.size(); ++index) {
        if (_M_edge_touched[index]) {
            _M_labels.reset(index);

            _M_base_labels[_M_graph.base_graph().source(index)] = none_value<N>;
            _M_base_labels[_M_graph.base_graph().destination(index)] = none_value<N>;
            _M_edge_touched[index] = false;
        }
    }
}

template<RoutableGraph G, typename N>
N
steiner_labels<G, N>::get(steiner_labels<G, N>::node_id_type node) const {
    assert(!is_none(node));
    if (_M_graph.is_base_node(node)) [[unlikely]]
        return _M_base_labels[_M_graph.base_node_id(node)];

    return _M_labels.node_info(node.edge, node.steiner_index - 1);
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::label(node_id_type node, N label) {
    auto edge_id = node.edge;

    assert(!is_none(node));

    if constexpr (requires(typename steiner_labels<G, N>::labels_type l, typename labels_type::edge_id_type e) {
        l.expand(e); l.is_expanded(e);
    }) {
        if (!_M_labels.is_expanded(edge_id)) [[unlikely]] {
            auto count = _M_graph.steiner_info(node.edge).node_count - 2;
            assert(count >= 0);
            assert(count > node.steiner_index - 1);

            _M_edge_touched[edge_id] = true;
            _M_labels.expand(edge_id, count);
        }
    } else {
        if (_M_graph.is_base_node(node) && is_infinity(_M_base_labels[_M_graph.base_node_id(node)].distance))
            _M_edge_touched[edge_id] = true;
        if (!_M_graph.is_base_node(node) && is_infinity(_M_labels.node_info(edge_id, node.steiner_index - 1).distance))
            _M_edge_touched[edge_id] = true;
    }

    if (_M_graph.is_base_node(node))
        [[unlikely]]
                _M_base_labels[_M_graph.base_node_id(node)] = label;
    else
        _M_labels.node_info(edge_id, node.steiner_index - 1) = label;
}
