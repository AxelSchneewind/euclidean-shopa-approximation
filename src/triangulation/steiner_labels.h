#include "steiner_graph.h"
#include "../graph/graph.h"

template<RoutableGraph G, typename NodeCostPair>
class steiner_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;
    using node_cost_pair_type = NodeCostPair;

    struct label {
        distance_type distance;
        node_id_type predecessor;
    };

    std::shared_ptr<const G> _M_graph;

    std::vector<typename G::triangle_edge_id_type> _M_touched;
    std::vector<std::vector<label>> _M_labels;

    static_assert(sizeof(std::vector<label>) == 3 * sizeof(nullptr));

public:
    explicit steiner_labels(std::shared_ptr<const G> __graph);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(node_id_type __node) const;

    distance_type distance(node_id_type __node) const;

    node_id_type predecessor(node_id_type __node) const;

    // TODO make iterator
    std::vector<node_id_type> all_visited() const;

    void label(node_cost_pair_type __node_cost_pair);
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
        _M_labels[edge].clear();
        _M_labels[edge].shrink_to_fit();
    }

    _M_touched.clear();
}

template<RoutableGraph Graph, typename N>
steiner_labels<Graph, N>::node_id_type
steiner_labels<Graph, N>::predecessor(steiner_labels<Graph, N>::node_id_type __node) const {
    assert(!is_none(__node));

    if (_M_labels[__node.edge].size() <= __node.steiner_index) {
        return none_value<node_id_type>();
    }
    return _M_labels[__node.edge][__node.steiner_index].predecessor;
}

template<RoutableGraph G, typename N>
G::distance_type
steiner_labels<G, N>::distance(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));
    if (_M_labels[__node.edge].size() <= __node.steiner_index) {
        return infinity<distance_type>();
    }
    return _M_labels[__node.edge][__node.steiner_index].distance;
}

template<RoutableGraph G, typename N>
bool
steiner_labels<G, N>::reached(steiner_labels<G, N>::node_id_type __node) const {
    assert(!is_none(__node));

    return _M_labels[__node.edge].size() > __node.steiner_index &&
           !is_none(_M_labels[__node.edge][__node.steiner_index].predecessor.edge);
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::label(steiner_labels<G, N>::node_cost_pair_type __node_cost_pair) {
    auto edge_id = __node_cost_pair.node.edge;
    auto count = _M_graph->steiner_info(__node_cost_pair.node.edge).node_count;

    assert(_M_touched.empty() ||
           _M_graph->topology().has_edge(__node_cost_pair.predecessor, __node_cost_pair.node));
    assert(__node_cost_pair.node.edge >= 0 && __node_cost_pair.node.edge < _M_graph->topology().edge_count());
    assert(count > 0);
    assert(count > __node_cost_pair.node.steiner_index);

    // if edge has not been touched yet, set up its distance/predecessor arrays
    if (_M_labels[edge_id].size() == 0) {
        _M_touched.push_back(edge_id);

        _M_labels[edge_id].resize(count, {infinity<distance_type>(), none_value<node_id_type>()});
    }

    _M_labels[edge_id][__node_cost_pair.node.steiner_index].distance = __node_cost_pair.distance;
    _M_labels[edge_id][__node_cost_pair.node.steiner_index].predecessor = __node_cost_pair.predecessor;
}
