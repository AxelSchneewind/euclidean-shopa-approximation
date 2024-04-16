#pragma once

#include "steiner_labels.h"
#include "../util/contract.h"


template<RoutableGraph G, typename Label>
steiner_labels<G, Label>::label_iterator_type
steiner_labels<G, Label>::all_visited() const {
    return label_iterator_type(_edge_touched.begin(), _edge_touched.end(),
                               std::function<steiner_graph::node_id_iterator_type(std::size_t)>(
                                       [this](std::size_t edge) -> steiner_graph::node_id_iterator_type {
                                           return _graph->node_ids(edge);
                                       }));
}

template<RoutableGraph G, typename Label>
steiner_labels<G, Label>::steiner_labels(steiner_labels &&other) noexcept
        : _graph{other._graph}
        , _edge_touched(std::move(other._edge_touched))
        , _base_labels{std::move(other._base_labels)}
        , _labels{std::move(other._labels)}
        , _default_value{other._default_value} {

}


template<RoutableGraph G, typename Label>
steiner_labels<G, Label>::steiner_labels(std::shared_ptr<G> graph, value_type default_value)
        : _graph{std::move(graph)}
        , _edge_touched(_graph->base_graph().edge_count(), false)
        , _base_labels{_graph->base_graph().node_count(), _default_value}
        , _labels{_graph->base_graph().edge_count()}
        , _default_value{default_value}
{
}

template<RoutableGraph G, typename Label>
void
steiner_labels<G, Label>::init(steiner_labels<G, Label>::node_id_type /*start_node*/,
                           steiner_labels<G, Label>::node_id_type /*target_node*/) {
    _labels.reset();
    std::fill(_base_labels.begin(), _base_labels.end(), _default_value);
    std::fill(_edge_touched.begin(), _edge_touched.end(), false);
}

template<RoutableGraph G, typename Label>
Label
steiner_labels<G, Label>::at(node_id_type node) const {
    assert(!optional::is_none(node));
    if (_graph->is_base_node(node)) [[unlikely]] {
        assert(!optional::is_none(_graph->base_node_id(node)));
        return _base_labels[_graph->base_node_id(node)];
    }

    assert(_labels.contains(node.edge));
    return _labels.at(node.edge, node.steiner_index - 1);
}

template<RoutableGraph G, typename Label>
Label& steiner_labels<G, Label>::at(node_id_type node) {
    assert(!optional::is_none(node));

    if (_graph->is_base_node(node)) [[unlikely]] {
        assert(!optional::is_none(_graph->base_node_id(node)));
        return _base_labels[_graph->base_node_id(node)];
    }

    assert (_labels.contains(node.edge));
    return _labels.at(node.edge, node.steiner_index - 1);
}


template<RoutableGraph G, typename Label>
Label& steiner_labels<G, Label>::operator[](node_id_type node) {
    assert(!optional::is_none(node));

    if (_graph->is_base_node(node)) [[unlikely]] {
        assert(!optional::is_none(_graph->base_node_id(node)));
        return _base_labels[_graph->base_node_id(node)];
    }

    if (!_labels.contains(node.edge)) {
        _edge_touched[node.edge] = true;
        _labels.append(node.edge, _graph->steiner_info(node.edge).node_count - 2, _default_value);
    }

    assert (_labels.contains(node.edge));
    return _labels.at(node.edge, node.steiner_index - 1);
}

template<RoutableGraph G, typename Label>
void steiner_labels<G, Label>::clear() {
    _labels.clear();
    std::fill(_base_labels.begin(), _base_labels.end(), _default_value);
}