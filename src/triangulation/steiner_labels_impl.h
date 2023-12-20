#pragma once

#include "steiner_labels.h"
#include "node_info_container.h"

template<RoutableGraph G, typename N>
steiner_labels<G, N>::label_iterator_type
steiner_labels<G, N>::all_visited() const {
    static decltype(_M_touched) copy;

    copy = _M_touched;

    remove_duplicates(copy);
    return label_iterator_type(copy.begin(), copy.end(),
                               std::function<steiner_graph::node_id_iterator_type(
                                       steiner_graph::triangle_edge_id_type)>(
                                       [this](steiner_graph::triangle_edge_id_type edge) -> steiner_graph::node_id_iterator_type {
                                           return _M_graph.node_ids(edge);
                                       }));
}

template<RoutableGraph G, typename N>
steiner_labels<G, N>::steiner_labels(G const &__graph)
        : _M_graph(__graph),
        //_M_labels(_M_graph.subdivision_info().offsets(), 0, none_value<N>),
          _M_labels(_M_graph.subdivision_info().offsets(), none_value<N>),
          _M_base_labels(_M_graph.base_graph().node_count(), none_value<N>) {
    _M_touched.reserve(10000);
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::init(steiner_labels<G, N>::node_id_type  /*__start_node*/,
                           steiner_labels<G, N>::node_id_type  /*__target_node*/) {
    for (size_t index = 0; index < _M_touched.size(); ++index) {
        typename G::triangle_edge_id_type edge = _M_touched[index];
        _M_labels.reset(edge);

        _M_base_labels[_M_graph.base_graph().source(edge)] = none_value<N>;
        _M_base_labels[_M_graph.base_graph().destination(edge)] = none_value<N>;
    }


    _M_touched.clear();
}

template<RoutableGraph G, typename N>
N
steiner_labels<G, N>::get(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));
    if (_M_graph.is_base_node(__node)) [[unlikely]]
        return _M_base_labels[_M_graph.base_node_id(__node)];

    return _M_labels.node_info(__node.edge, __node.steiner_index - 1);
}

template<RoutableGraph G, typename N>
bool
steiner_labels<G, N>::reached(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));

    if (_M_graph.is_base_node(__node)) [[unlikely]]
        return _M_base_labels[_M_graph.base_node_id(__node)].distance != infinity<distance_t>;
    return _M_labels.node_info(__node.edge, __node.steiner_index - 1).distance != infinity<distance_t>;
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::label(node_id_type __node, N __label) {
    auto edge_id = __node.edge;

    assert(!is_none(__node));

    if constexpr (requires(typename steiner_labels<G, N>::labels_type l, typename labels_type::edge_id_type e) {
        l.expand(e); l.is_expanded(e);
    }) {
        if (!_M_labels.is_expanded(edge_id)) [[unlikely]] {
            auto count = _M_graph.steiner_info(__node.edge).node_count - 2;
            assert(count >= 0);
            assert(count > __node.steiner_index - 1);

            _M_touched.push_back(edge_id);
            _M_labels.expand(edge_id, count);
        }
    } else {
        if (!reached(__node)) [[unlikely]] {
            _M_touched.push_back(edge_id);
        }
    }

    if (_M_graph.is_base_node(__node))
        [[unlikely]]
                _M_base_labels[_M_graph.base_node_id(__node)] = __label;
    else
        _M_labels.node_info(edge_id, __node.steiner_index - 1) = __label;
}
