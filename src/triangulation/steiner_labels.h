#include "steiner_graph.h"
#include "../graph/graph.h"

template<RoutableGraph G, typename NodeCostPair>
class steiner_labels {
private:
    using node_id_type = G::node_id_type;
    using distance_type = G::distance_type;
    using node_cost_pair_type = NodeCostPair;

    const G *_M_graph_ptr;

    std::vector<typename G::triangle_edge_id_type> _M_touched;
    std::vector<std::vector<distance_type>> _M_distance;
    std::vector<std::vector<node_id_type>> _M_predecessor;

public:
    explicit steiner_labels(const G *__graph_ptr);

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node);

    bool reached(const node_id_type &__node) const;

    distance_type distance(const node_id_type &__node) const;

    node_id_type predecessor(const node_id_type &__node) const;

    std::vector<node_id_type> all_visited() const;

    void label(const node_cost_pair_type &__node_cost_pair);
};


template<RoutableGraph G, typename N>
std::vector<typename steiner_labels<G, N>::node_id_type>
steiner_labels<G, N>::all_visited() const {
    std::vector<typename steiner_labels<G, N>::node_id_type> result;

    for (auto edge: _M_touched) {
        for (auto node: _M_graph_ptr->node_ids(edge)) {
            if (reached(node)) {
                result.emplace_back(node);
            }
        }
    }

    return result;
}


template<RoutableGraph G, typename N>
steiner_labels<G, N>::steiner_labels(const G *__graph_ptr)
        : _M_graph_ptr(__graph_ptr),
          _M_predecessor(__graph_ptr->base_graph().edge_count()),
          _M_distance(__graph_ptr->base_graph().edge_count()) {
    _M_touched.reserve(std::sqrt(__graph_ptr->base_graph().edge_count()));
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::init(steiner_labels<G, N>::node_id_type  /*__start_node*/,
                           steiner_labels<G, N>::node_id_type  /*__target_node*/) {
    for (size_t index = 0; index < _M_touched.size(); ++index) {
        steiner_labels<G, N>::node_id_type node = _M_touched[index];
        _M_predecessor[node].clear();
        _M_distance[node].clear();
        _M_predecessor[node].shrink_to_fit();
        _M_distance[node].shrink_to_fit();
    }

    _M_touched.clear();
}

template<RoutableGraph Graph, typename N>
steiner_labels<Graph, N>::node_id_type
steiner_labels<Graph, N>::predecessor(const steiner_labels<Graph, N>::node_id_type &__node) const {
    assert(__node);
    if (_M_predecessor[__node.edge].size() <= __node.steiner_index) {
        return {};
    }
    return _M_predecessor[__node.edge][__node.steiner_index];
}

template<RoutableGraph G, typename N>
G::distance_type
steiner_labels<G, N>::distance(const steiner_labels<G, N>::node_id_type &__node) const {
    assert(__node);
    if (_M_distance[__node.edge].size() <= __node.steiner_index) {
        return {};
    }
    return _M_distance[__node.edge][__node.steiner_index];
}

template<RoutableGraph G, typename N>
bool
steiner_labels<G, N>::reached(const steiner_labels<G, N>::node_id_type &__node) const {
    assert(__node);
    return _M_predecessor[__node.edge].size() > __node.steiner_index &&
           _M_predecessor[__node.edge][__node.steiner_index].edge;
}

template<RoutableGraph G, typename N>
void
steiner_labels<G, N>::label(const steiner_labels<G, N>::node_cost_pair_type &__node_cost_pair) {
    // TODO avoid
    if(__node_cost_pair.node == __node_cost_pair.predecessor) return;

    auto edge_id = __node_cost_pair.node.edge;
    auto inv_edge_id = _M_graph_ptr->base_graph().edge_id(_M_graph_ptr->base_graph().destination(edge_id),
                                                          _M_graph_ptr->base_graph().source(edge_id));
    auto count = _M_graph_ptr->steiner_info(__node_cost_pair.node.edge).node_count;
    if (!reached(__node_cost_pair.node)) {
        _M_touched.push_back(edge_id);
        _M_touched.push_back(inv_edge_id);

        _M_distance[edge_id].resize(count, {});
        _M_distance[inv_edge_id].resize(count, {});
        _M_predecessor[edge_id].resize(count, {});
        _M_predecessor[inv_edge_id].resize(count, {});
    }

    _M_distance[edge_id][__node_cost_pair.node.steiner_index] = __node_cost_pair.distance;
    _M_predecessor[edge_id][__node_cost_pair.node.steiner_index] = __node_cost_pair.predecessor;
    _M_distance[inv_edge_id][count - __node_cost_pair.node.steiner_index - 1] = __node_cost_pair.distance;
    _M_predecessor[inv_edge_id][count - __node_cost_pair.node.steiner_index - 1] = __node_cost_pair.predecessor;

}
