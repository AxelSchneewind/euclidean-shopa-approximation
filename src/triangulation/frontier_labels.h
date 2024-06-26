#pragma once

#include "steiner_graph.h"

#include "../routing/dijkstra_concepts.h"

#include <queue>
#include <vector>
#include <memory>
#include <map>


template<typename T>
concept DistanceNodeCostPair = HasDistance<T> && HasNode<T>;

/**
 * only stores labels for relevant nodes in a graph with steiner points
 * @tparam NodeCostPair
 */
template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
class frontier_labels {
public:
    using node_id_type = typename Graph::node_id_type;
    using base_node_id_type = typename Graph::triangle_node_id_type;
    using base_edge_id_type = typename Graph::triangle_edge_id_type;
    using distance_type = typename Graph::distance_type;
    using node_cost_pair_type = NodeCostPair;

    using label_type = Label;
    using value_type = Label;

private:
    struct aggregate_info {
        std::vector<label_type> labels;
    };


    struct aggregate_throwaway_info {
        base_edge_id_type edge;
        typename Graph::distance_type throwaway_distance;
    };

    struct compare_aggregate_throwaway {
        bool operator()(aggregate_throwaway_info const &e1, aggregate_throwaway_info const &e2) {
            return e1.throwaway_distance > e2.throwaway_distance;
        }
    };


    std::shared_ptr<Graph> _graph;

    std::unordered_map<base_node_id_type, aggregate_info> _expanded_node_aggregates;
    // compact_node_info_container<base_edge_id_type, short unsigned int, std::nullptr_t, label_type> _expanded_node_aggregates;

    std::priority_queue<aggregate_throwaway_info, std::vector<aggregate_throwaway_info>, compare_aggregate_throwaway> _active_aggregates;

    label_type _default_value;

    // nodes with lower value (minimal path length) will not be labelled again, assuming nodes are labelled with ascending value()
    distance_type _min_value;
    // nodes with higher distance are preliminarily labelled
    distance_type _max_distance;

    // nodes with value < min_value - frontier_width can be discarded, this value has to be lower than the maximum edge length
    distance_type _frontier_width;

public:

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::shared_ptr<aggregate_info>);

    // ~4000 for wgs projection, 3.0 for latlon
    frontier_labels(std::shared_ptr<Graph> graph, label_type default_value = optional::none_value<label_type>);

    frontier_labels(frontier_labels &&) noexcept = default;

    frontier_labels &operator=(frontier_labels &&) noexcept = default;

    ~frontier_labels() = default;

    size_t aggregate_count() const;

    // init for given query
    void init(node_id_type start_node, node_id_type target_node);

    bool contains(node_id_type node) const;

    label_type const &at(node_id_type node) const;

    label_type &at(node_id_type node);

    label_type const &operator[](node_id_type node) const;

    label_type &operator[](node_id_type node);

    /**
     * informs the data structure that node information with distance less than the given one can be discarded
     * @param new_distance
     */
    void set_frontier_distance(distance_type new_distance);

    void set_frontier_width(distance_type new_width);
};


template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<Graph, NodeCostPair, Label>::set_frontier_width(distance_type new_width) {
    _frontier_width = new_width;
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<Graph, NodeCostPair, Label>::set_frontier_distance(distance_type new_distance) {
    _min_value = std::max(_min_value, new_distance);

    while (!_active_aggregates.empty() && _active_aggregates.top().throwaway_distance < _min_value - _frontier_width) {
        _expanded_node_aggregates.erase(_active_aggregates.top().edge);
        _active_aggregates.pop();
    }
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
Label const &frontier_labels<Graph, NodeCostPair, Label>::operator[](node_id_type node) const {
    if (!_expanded_node_aggregates.contains(node.edge)) {
        assert(false);
    }
    return _expanded_node_aggregates.at(node.edge).labels[node.steiner_index];
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
Label &frontier_labels<Graph, NodeCostPair, Label>::operator[](node_id_type node) {
    if (!_expanded_node_aggregates.contains(node.edge)) {
        _expanded_node_aggregates[node.edge] = {
                std::vector<Label>(_graph->steiner_info(node.edge).node_count, _default_value)};
        _active_aggregates.push({node.edge, _min_value});
    }
    return _expanded_node_aggregates.at(node.edge).labels[node.steiner_index];
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
Label const &frontier_labels<Graph, NodeCostPair, Label>::at(node_id_type node) const {
    return _expanded_node_aggregates.at(node.edge).labels[node.steiner_index];
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
Label &frontier_labels<Graph, NodeCostPair, Label>::at(node_id_type node) {
    return _expanded_node_aggregates.at(node.edge).labels[node.steiner_index];
}


template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
bool frontier_labels<Graph, NodeCostPair, Label>::contains(node_id_type node) const {
    // return _expanded_node_aggregates.node_count(node.edge) > 0 &&
    //        !is_infinity(_expanded_node_aggregates.at(node.edge, node.steiner_index).distance());
    return _expanded_node_aggregates.contains(node.edge)
           && node.steiner_index >= 0
           && _expanded_node_aggregates.at(node.edge).labels.size() > static_cast<size_t>(node.steiner_index)
           && (_expanded_node_aggregates.at(node.edge).labels[node.steiner_index].value() != _default_value.value());
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
void frontier_labels<Graph, NodeCostPair, Label>::init(node_id_type start_node,
                                                       node_id_type target_node) {
    // _expanded_node_aggregates.reset();
    _expanded_node_aggregates.clear();

    while (!_active_aggregates.empty())
        _active_aggregates.pop();
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
size_t frontier_labels<Graph, NodeCostPair, Label>::aggregate_count() const {
    // return _expanded_node_aggregates.edge_count();
    return _expanded_node_aggregates.size();
}

template<typename Graph, DistanceNodeCostPair NodeCostPair, HasDistance Label>
frontier_labels<Graph, NodeCostPair, Label>::frontier_labels(std::shared_ptr<Graph> graph,
                                                             label_type default_value)
        : _graph(graph),
        // _expanded_node_aggregates{graph->subdivision_info().offsets(), nullptr, default_value},
          _expanded_node_aggregates{},
          _active_aggregates{},
          _default_value{default_value},
          _min_value{0.0},
          _max_distance{0.0},
          _frontier_width{0.0} {
    // use highest edge distance, doubled as frontier_width
    for (auto &&e: graph->base_graph().edge_ids()) {
        _frontier_width = std::max(_frontier_width, distance(graph->node_coordinates(graph->base_graph().source(e)),
                                                             graph->node_coordinates(
                                                                     graph->base_graph().destination(e))));
    }
    _frontier_width *= 2.0;
    std::cerr << "frontier width: " << _frontier_width << '\n';
}
