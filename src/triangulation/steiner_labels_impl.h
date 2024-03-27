#pragma once

#include "steiner_labels.h"
#include "../util/contract.h"


template<RoutableGraph G, typename Label>
steiner_labels<G, Label>::steiner_labels(steiner_labels &&other) noexcept
        : _graph(other._graph), _edge_touched(std::move(other._edge_touched)),
          _base_labels(std::move(other._base_labels)), _labels(std::move(other._labels)) {}

template<RoutableGraph G, typename N>
steiner_labels<G, N>::label_iterator_type
steiner_labels<G, N>::all_visited() const {
    return label_iterator_type(_edge_touched.begin(), _edge_touched.end(),
                               std::function<steiner_graph::node_id_iterator_type(std::size_t)>(
                                       [this](std::size_t edge) -> steiner_graph::node_id_iterator_type {
                                           return _graph->node_ids(edge);
                                       }));
}

template<RoutableGraph G, typename N>
steiner_labels<G, N>::steiner_labels(std::shared_ptr<G> graph)
        : _graph(std::move(graph))
        , _edge_touched(_graph->base_graph().edge_count(), false)
        , _base_labels(_graph->base_graph().node_count(), optional::none_value<N>)
        , _labels(_graph->base_graph().edge_count(), optional::none_value<N>)
{
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::init(steiner_labels<G, N>::node_id_type /*start_node*/,
                           steiner_labels<G, N>::node_id_type /*target_node*/) {
    _labels.reset();
    std::fill(_base_labels.begin(), _base_labels.end(), (optional::none_value<N>));
    std::fill(_edge_touched.begin(), _edge_touched.end(), false);
}

template<RoutableGraph G, typename N>
N
steiner_labels<G, N>::at(steiner_labels<G, N>::node_id_type node) const {
    assert(!optional::is_none(node));
    if (_graph->is_base_node(node)) [[unlikely]] {
        assert(!optional::is_none(_graph->base_node_id(node)));
        return _base_labels[_graph->base_node_id(node)];
    }

    assert(_labels.contains(node.edge));
    return _labels.at(node.edge, node.steiner_index - 1);
}

template<RoutableGraph G, typename N>
N &
steiner_labels<G, N>::at(steiner_labels<G, N>::node_id_type node) {
    assert(!optional::is_none(node));

    if (_graph->is_base_node(node)) [[unlikely]] {
        assert(!optional::is_none(_graph->base_node_id(node)));
        return _base_labels[_graph->base_node_id(node)];
    }

    assert (_labels.contains(node.edge));
    return _labels.at(node.edge, node.steiner_index - 1);
}


template<RoutableGraph G, typename Label>
Label &steiner_labels<G, Label>::operator[](const node_id_type &node) {
    assert(!optional::is_none(node));

    if (_graph->is_base_node(node)) [[unlikely]] {
        assert(!optional::is_none(_graph->base_node_id(node)));
        return _base_labels[_graph->base_node_id(node)];
    }

    if (!_labels.contains(node.edge)) {
        _edge_touched[node.edge] = true;
        _labels.append(node.edge, _graph->steiner_info(node.edge).node_count - 2);
    }

    assert (_labels.contains(node.edge));
    return _labels.at(node.edge, node.steiner_index - 1);
}
