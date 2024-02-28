#pragma once

#include "steiner_graph.h"

#include "../routing/dijkstra_concepts.h"

#include <queue>
#include <vector>


template<typename T>
concept DistanceNodeCostPair = HasDistance<T> && HasNode<T>;

/**
 * only stores labels for relevant nodes in a graph with steiner points
 * @tparam NodeCostPair
 */
template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
class frontier_labels {
public:
    using node_id_type = steiner_graph::node_id_type;
    using base_node_id_type = steiner_graph::triangle_node_id_type;
    using base_edge_id_type = steiner_graph::triangle_edge_id_type;
    using distance_type = steiner_graph::distance_type;
    using node_cost_pair_type = NodeCostPair;

    using label_type = Label;

private:
    struct aggregate_info {
        std::vector<label_type> labels;
    };


    struct aggregate_throwaway_info {
        base_edge_id_type edge;
        steiner_graph::distance_type throwaway_distance;
    };

    struct compare_aggregate_throwaway {
        bool operator()(aggregate_throwaway_info const &e1, aggregate_throwaway_info const &e2) {
            return e1.throwaway_distance > e2.throwaway_distance;
        }
    };


    steiner_graph const &_graph;

    // nodes with lower value (minimal path length) will not be labelled again, assuming nodes are labelled with ascending value()
    distance_type _min_value;
    // nodes with higher distance are preliminarily labelled
    distance_type _max_distance;

    // nodes with value < min_value - frontier_width can be discarded, this value has to be lower than the maximum edge length
    distance_type _frontier_width;

    // std::unordered_map<base_node_id_type, std::shared_ptr<aggregate_info>> _expanded_node_aggregates;
    compact_node_info_container<base_edge_id_type, short unsigned int, nullptr_t, label_type> _expanded_node_aggregates;

    label_type _default_value;

    std::priority_queue<aggregate_throwaway_info, std::vector<aggregate_throwaway_info>, compare_aggregate_throwaway> _active_aggregates;

public:

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::shared_ptr<aggregate_info>);

    frontier_labels(steiner_graph const &graph, distance_type frontier_width = 0.2,
                    label_type default_value = optional::none_value<label_type>);

    frontier_labels(frontier_labels &&) noexcept = default;

    frontier_labels &operator=(frontier_labels &&) noexcept = default;

    ~frontier_labels() = default;

    size_t aggregate_count() const;

    // init for given query
    void init(node_id_type start_node, node_id_type target_node);;

    bool reached(node_id_type node) const;

    label_type get(node_id_type node) const;

    label_type get_preliminary(node_id_type node) const;

    /**
     * informs the data structure that node information with distance less than the given one can be discarded
     * @param new_distance
     */
    void set_frontier_distance(distance_type new_distance);

    void set_frontier_width(distance_type new_width);

    void label_preliminary(steiner_graph::node_id_type node, node_cost_pair_type node_cost_pair);

    void label(steiner_graph::node_id_type node, node_cost_pair_type node_cost_pair);;
};

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<NodeCostPair, Label>::label(steiner_graph::node_id_type node, node_cost_pair_type node_cost_pair) {
    label_preliminary(node, node_cost_pair);
    _max_distance = std::max(_max_distance, node_cost_pair.distance);
    set_frontier_distance(node_cost_pair.value() - _frontier_width);
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<NodeCostPair, Label>::label_preliminary(steiner_graph::node_id_type node,
                                                             node_cost_pair_type node_cost_pair) {
    if (_expanded_node_aggregates.node_info(node.edge, node.steiner_index).distance >
        node_cost_pair.distance)
        _expanded_node_aggregates.node_info(node.edge, node.steiner_index) = node_cost_pair;
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<NodeCostPair, Label>::set_frontier_width(frontier_labels::distance_type new_width) {
    _frontier_width = new_width;
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<NodeCostPair, Label>::set_frontier_distance(frontier_labels::distance_type new_distance) {
    _min_value = std::max(_min_value, new_distance);

    while (!_active_aggregates.empty() && _active_aggregates.top().throwaway_distance < _min_value) {
        _expanded_node_aggregates.erase(_active_aggregates.top().edge);
        _active_aggregates.pop();
    }
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
Label frontier_labels<NodeCostPair, Label>::get_preliminary(frontier_labels::node_id_type node) const {
    return _expanded_node_aggregates.node_info(node.edge, node.steiner_index);
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
Label frontier_labels<NodeCostPair, Label>::get(frontier_labels::node_id_type node) const {
    return get_preliminary(node);
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
bool frontier_labels<NodeCostPair, Label>::reached(frontier_labels::node_id_type node) const {
    return _expanded_node_aggregates.node_count(node.edge) > 0 &&
           !is_infinity(_expanded_node_aggregates.node_info(node.edge, node.steiner_index).distance());
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<NodeCostPair, Label>::init(frontier_labels::node_id_type start_node,
                                                frontier_labels::node_id_type target_node) {
    _expanded_node_aggregates.reset();

    while (!_active_aggregates.empty())
        _active_aggregates.pop();
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
size_t frontier_labels<NodeCostPair, Label>::aggregate_count() const {
    return _expanded_node_aggregates.edge_count();
}

template<DistanceNodeCostPair NodeCostPair, HasDistance Label>
frontier_labels<NodeCostPair, Label>::frontier_labels(const steiner_graph &graph,
                                                      frontier_labels::distance_type frontier_width,
                                                      label_type default_value)
        : _graph(graph),
          _expanded_node_aggregates{graph.subdivision_info().offsets(), nullptr, default_value},
          _min_value{0.0},
          _max_distance{0.0},
          _default_value(default_value),
          _frontier_width(frontier_width) {}