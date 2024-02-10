#pragma once

#include "steiner_graph.h"
#include "../graph/graph.h"

#include "../routing/dijkstra_concepts.h"
#include "node_info_array.h"

#include <queue>


template<typename T>
concept DistanceNodeCostPair = HasDistance<T> && HasNode<T>;


// TODO move somewhere else
struct distance_label {
public:
    using distance_type = steiner_graph::distance_type;
private:
    distance_type _distance;
public:

    static constexpr steiner_graph::node_id_type predecessor = none_value<steiner_graph::node_id_type>;

    constexpr distance_label() : _distance(infinity<distance_type>) {}

    constexpr distance_label(distance_type dist) : _distance(dist) {}

    template<typename NCP>
    requires HasDistance<NCP>
    constexpr distance_label(NCP ncp) : _distance(ncp.distance()) {}

    constexpr distance_label(distance_type distance, steiner_graph::node_id_type) : _distance(distance) {}

    template<typename NCP>
    requires HasDistance<NCP>
    operator NCP() {
        return {
                none_value<steiner_graph::node_id_type>,
                none_value<steiner_graph::node_id_type>,
                _distance,
        };
    }

    distance_type& distance() { return _distance; }
    distance_type const& distance() const { return _distance; }
};

template<>
constexpr distance_label none_value<distance_label> = {infinity<steiner_graph::distance_type>,
                                                       none_value<steiner_graph::node_id_type>};


/**
 * only stores labels for relevant nodes in a graph with steiner points
 * @tparam NodeCostPair
 */
template<DistanceNodeCostPair NodeCostPair, HasDistance Label = distance_label>
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


    steiner_graph const &_M_graph;

    // nodes with lower value (minimal path length) will not be labelled again, assuming nodes are labelled with ascending value()
    distance_type min_value;
    // nodes with higher distance are preliminarily labelled
    distance_type max_distance;

    // nodes with value < min_value - frontier_width can be discarded, this value has to be lower than the maximum edge length
    distance_type frontier_width;

    // std::unordered_map<base_node_id_type, std::shared_ptr<aggregate_info>> _M_expanded_node_aggregates;
    compact_node_info_container<base_edge_id_type, short unsigned int, nullptr_t, label_type> _M_expanded_node_aggregates;

    label_type default_value;

    std::priority_queue<aggregate_throwaway_info, std::vector<aggregate_throwaway_info>, compare_aggregate_throwaway> _M_active_aggregates;

public:

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::shared_ptr<aggregate_info>);

    frontier_labels(steiner_graph const &__graph, distance_type frontier_width = 0.2,
                    label_type default_value = none_value<label_type>)
            : _M_graph(__graph),
              _M_expanded_node_aggregates{__graph.subdivision_info().offsets(), nullptr, default_value}, min_value{0.0},
              max_distance{0.0}, default_value(default_value), frontier_width(frontier_width) {};

    frontier_labels(frontier_labels &&) noexcept = default;

    frontier_labels &operator=(frontier_labels &&) noexcept = default;

    ~frontier_labels() = default;

    size_t aggregate_count() const {
        return _M_expanded_node_aggregates.edge_count();
    }

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node) {
        _M_expanded_node_aggregates.reset();

        while (!_M_active_aggregates.empty())
            _M_active_aggregates.pop();
    };

    bool reached(node_id_type __node) const {
        return _M_expanded_node_aggregates.node_count(__node.edge) > 0 &&
               !is_infinity(_M_expanded_node_aggregates.node_info(__node.edge, __node.steiner_index).distance());
    }

    label_type get(node_id_type __node) const {
        return get_preliminary(__node);
    }

    label_type get_preliminary(node_id_type __node) const {
        return _M_expanded_node_aggregates.node_info(__node.edge, __node.steiner_index);
    }

    /**
     * informs the data structure that node information with distance less than the given one can be discarded
     * @param new_distance
     */
    void set_frontier_distance(distance_type new_distance) {
        min_value = std::max(min_value, new_distance);

        while (!_M_active_aggregates.empty() && _M_active_aggregates.top().throwaway_distance < min_value) {
            _M_expanded_node_aggregates.erase(_M_active_aggregates.top().edge);
            _M_active_aggregates.pop();
        }
    }

    void set_frontier_width(distance_type new_width) {
        frontier_width = new_width;
    }

    void label_preliminary(steiner_graph::node_id_type __node, node_cost_pair_type __node_cost_pair) {
        if (_M_expanded_node_aggregates.node_info(__node.edge, __node.steiner_index).distance >
            __node_cost_pair.distance)
            _M_expanded_node_aggregates.node_info(__node.edge, __node.steiner_index) = __node_cost_pair;
    }

    void label(steiner_graph::node_id_type __node, node_cost_pair_type __node_cost_pair) {
        label_preliminary(__node, __node_cost_pair);
        max_distance = std::max(max_distance, __node_cost_pair.distance);
        set_frontier_distance(__node_cost_pair.value() - frontier_width);
    };
};

