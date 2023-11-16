#pragma once

#include "steiner_graph.h"
#include "../graph/graph.h"


template<RoutableGraph G, typename Label>
class steiner_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;

    std::shared_ptr<const G> _M_graph;

    std::vector<typename G::triangle_edge_id_type> _M_touched;
    std::vector<std::shared_ptr<std::vector<Label>>> _M_labels;  // TODO

public:
    using label_type = Label;

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::shared_ptr<std::vector<Label>>);

    explicit steiner_labels(std::shared_ptr<const G> __graph);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(node_id_type __node) const;

    Label get(node_id_type __node) const;

    // TODO make iterator
    std::vector<node_id_type> all_visited() const;

    void label(node_id_type __node, Label __label);
};


template<RoutableGraph G, typename N>
std::vector<typename steiner_labels<G, N>::node_id_type>
steiner_labels<G, N>::all_visited() const {
    std::vector<typename steiner_labels<G, N>::node_id_type> result;

    for (auto edge: _M_touched) {
        for (auto node: _M_graph->node_ids(edge)) {
            if (!is_none(node.edge) && node.steiner_index != -1 && reached(node)) {
                result.emplace_back(node);
            }
        }
    }

    return result;
}


template<RoutableGraph G, typename N>
steiner_labels<G, N>::steiner_labels(std::shared_ptr<const G> __graph)
        : _M_graph(__graph),
          _M_labels(_M_graph->base_graph().edge_count()) {
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
        return none_value<N>();
    }
    return (*_M_labels[__node.edge])[__node.steiner_index];
}

template<RoutableGraph G, typename N>
bool
steiner_labels<G, N>::reached(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));

    return _M_labels[__node.edge] &&
           !is_none((*_M_labels[__node.edge])[__node.steiner_index].predecessor.edge);
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::label(node_id_type __node, N __label) {
    auto edge_id = __node.edge;
    auto count = _M_graph->steiner_info(__node.edge).node_count;

    assert(_M_touched.empty() ||
           _M_graph->topology().has_edge(__label.predecessor, __node));
    assert(__node.edge >= 0 && __node.edge < _M_graph->topology().edge_count());
    assert(count > 0);
    assert(count > __node.steiner_index);

    // if edge has not been touched yet, set up its distance/predecessor arrays
    if (!_M_labels[edge_id]) {
        _M_touched.push_back(edge_id);
        _M_labels[edge_id] = std::make_shared<std::vector<N>>(count);
    }

    (*_M_labels[edge_id])[__node.steiner_index] = __label;
}
