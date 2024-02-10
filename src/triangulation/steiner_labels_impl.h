#pragma once

#include "steiner_labels.h"
#include "node_info_container.h"

template<RoutableGraph G, typename N>
steiner_labels<G, N>::label_iterator_type
steiner_labels<G, N>::all_visited() const {
    return label_iterator_type(_edge_touched.begin(), _edge_touched.end(),
                               std::function<steiner_graph::node_id_iterator_type(std::size_t)>(
                                   [this](std::size_t edge) -> steiner_graph::node_id_iterator_type {
                                       return _graph.node_ids(edge);
                                   }));
}

template<RoutableGraph G, typename N>
steiner_labels<G, N>::steiner_labels(G const&graph)
    : _graph(graph),
      _labels(_graph.subdivision_info().offsets(), {none_value<node_id_type>}, none_value<N>),
      //_labels(_graph.subdivision_info().offsets(), none_value<N>),
      _base_labels(_graph.base_graph().node_count(), none_value<N>),
      _edge_touched(_graph.base_graph().edge_count(), false) {
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::init(steiner_labels<G, N>::node_id_type /*start_node*/,
                           steiner_labels<G, N>::node_id_type /*target_node*/) {
    for (size_t index = 0; index < _edge_touched.size(); ++index) {
        if (_edge_touched[index]) {
            _labels.reset(index);

            _base_labels[_graph.base_graph().source(index)] = none_value<N>;
            _base_labels[_graph.base_graph().destination(index)] = none_value<N>;
            _edge_touched[index] = false;
        }
    }
}

template<RoutableGraph G, typename N>
N const&
steiner_labels<G, N>::at(steiner_labels<G, N>::node_id_type node) const {
    assert(!is_none(node));
    if (_graph.is_base_node(node)) [[unlikely]]
    {
        assert(!is_none(_graph.base_node_id(node)));
        return _base_labels[_graph.base_node_id(node)];
    }

    return _labels.node_info(node.edge, node.steiner_index - 1);
}

template<RoutableGraph G, typename Label>
typename steiner_labels<G, Label>::edge_label_type const& steiner_labels<G, Label>::at(edge_id_type edge) const {
    return _labels.get_aggregate_info(edge);
}

template<RoutableGraph G, typename Label>
typename steiner_labels<G, Label>::edge_label_type& steiner_labels<G, Label>::at(edge_id_type edge) {
    return _labels.get_aggregate_info(edge);
}

template<RoutableGraph G, typename N>
N &
steiner_labels<G, N>::at(steiner_labels<G, N>::node_id_type node) {
    assert(!is_none(node));
    if (_graph.is_base_node(node)) [[unlikely]]
    {
        assert(!is_none(_graph.base_node_id(node)));
        return _base_labels[_graph.base_node_id(node)];
    }

    return _labels.node_info(node.edge, node.steiner_index - 1);
}


template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::label(node_id_type const&node, N const&label) {
    auto const&edge_id = node.edge;

    assert(!is_none(node));

    if constexpr (requires(typename steiner_labels<G, N>::labels_type l, typename labels_type::edge_id_type e)
    {
        l.expand(e); l.is_expanded(e);
    }) {
        if (!_labels.is_expanded(edge_id)) [[unlikely]]
        {
            auto count = _graph.steiner_info(node.edge).node_count - 2;
            assert(count >= 0);
            assert(count > node.steiner_index - 1);

            _edge_touched[edge_id] = true;
            _labels.expand(edge_id, count);
        }
    }
    else {
        _edge_touched[edge_id] = true;
    }

    if (_graph.is_base_node(node)) { [[unlikely]]
        assert(!is_none(_graph.base_node_id(node)));
        _base_labels[_graph.base_node_id(node)] = label;
    } else
        _labels.node_info(edge_id, node.steiner_index - 1) = label;
}
