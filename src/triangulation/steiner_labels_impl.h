#pragma once

#include "steiner_labels.h"
#include "node_info_container.h"

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
          _M_labels(_M_graph.subdivision_info().offsets(), 0, none_value<N>),
          // _M_labels(_M_graph.subdivision_info().offsets(), none_value<N>),
          _M_base_labels(_M_graph.base_graph().node_count(), none_value<N>)
{
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
    if (_M_graph.is_base_node(__node))
        return _M_base_labels[_M_graph.base_node_id(__node)];

    return _M_labels.node_info(__node.edge, __node.steiner_index);
}

template<RoutableGraph G, typename N>
bool
steiner_labels<G, N>::reached(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));

    if (_M_graph.is_base_node(__node))
        return _M_base_labels[_M_graph.base_node_id(__node)].distance != infinity<distance_t>;
    return _M_labels.node_info(__node.edge, __node.steiner_index).distance != infinity<distance_t>;
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

    if (!reached(__node))
        _M_touched.push_back(__node.edge);

    if (_M_graph.is_base_node(__node))
        _M_base_labels[_M_graph.base_node_id(__node)] = __label;
    else
        _M_labels.node_info(edge_id,__node.steiner_index) = __label;
}
