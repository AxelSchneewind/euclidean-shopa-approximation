#pragma once

#include "steiner_graph.h"
#include "../graph/graph.h"

#include "../routing/dijkstra_concepts.h"

#include <queue>


template<typename T>
concept DistanceNodeCostPair = HasDistance<T, steiner_graph::distance_type> && HasNode<T, steiner_graph::node_id_type>;


// TODO move somewhere else
struct distance_label {
    using distance_type = steiner_graph::distance_type;

    distance_type distance;
    static constexpr steiner_graph::node_id_type predecessor = none_value<steiner_graph::node_id_type>;

    constexpr distance_label() : distance(infinity<distance_type>) {}

    constexpr distance_label(distance_type dist) : distance(dist) {}

    template<typename NCP>
    requires HasDistance<NCP, distance_type>
    constexpr distance_label(NCP ncp) : distance(ncp.distance) {}

    constexpr distance_label(distance_type distance, steiner_graph::node_id_type) : distance(distance) {}

    template<typename NCP>
    requires HasDistance<NCP, distance_type>
    operator NCP() {
        return {
                none_value<steiner_graph::node_id_type>,
                none_value<steiner_graph::node_id_type>,
                distance,
        };
    }
};

template<>
constexpr distance_label none_value<distance_label> = {infinity<steiner_graph::distance_type>,
                                                       none_value<steiner_graph::node_id_type>};


/**
 * only stores labels for relevant nodes in a graph with steiner points
 * @tparam NodeCostPair
 */
template<DistanceNodeCostPair NodeCostPair, HasDistance<steiner_graph::distance_type> Label = distance_label>
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

    std::unordered_map<base_node_id_type, std::shared_ptr<aggregate_info>> _M_expanded_node_aggregates;

    label_type default_value;

    std::priority_queue<aggregate_throwaway_info, std::vector<aggregate_throwaway_info>, compare_aggregate_throwaway> _M_active_aggregates;

public:

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::shared_ptr<aggregate_info>);

    frontier_labels(steiner_graph const &__graph, distance_type frontier_width = 0.1,
                    label_type default_value = none_value<label_type>) : _M_graph(
            __graph), _M_expanded_node_aggregates{}, min_value{0.0}, max_distance{0.0}, default_value(default_value),
                                                                         frontier_width(frontier_width) {};

    size_t aggregate_count() const {
        return _M_expanded_node_aggregates.size();
    }

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node) {
        _M_expanded_node_aggregates.clear();

        while (!_M_active_aggregates.empty())
            _M_active_aggregates.pop();
    };

    bool reached(node_id_type __node) const {
        return _M_expanded_node_aggregates.contains(__node.edge) &&
               _M_expanded_node_aggregates.at(__node.edge)->labels[__node.steiner_index].distance <= max_distance;
    }

    label_type get(node_id_type __node) const {
        if (reached(__node)) [[likely]]
            return get_preliminary(__node);
        else
            return default_value;
    }

    label_type get_preliminary(node_id_type __node) const {
        if (_M_expanded_node_aggregates.contains(__node.edge)) [[likely]]
            return _M_expanded_node_aggregates.at(__node.edge)->labels[__node.steiner_index];
        else
            return default_value;
    }

    bool active(base_edge_id_type __edge) const {
        return _M_expanded_node_aggregates.contains(__edge);
    };

    /**
     * informs the data structure that node information with distance less than the given one can be discarded
     * @param new_distance
     */
    void set_frontier_distance(distance_type new_distance) {
        min_value = std::max(min_value, new_distance);

        while (_M_active_aggregates.top().throwaway_distance < min_value) {
            _M_expanded_node_aggregates.erase(_M_active_aggregates.top().edge);
            _M_active_aggregates.pop();
        }
    }

    void label_preliminary(steiner_graph::node_id_type __node, node_cost_pair_type __node_cost_pair) {
        int steiner_index = __node.steiner_index;
        base_edge_id_type edge = __node.edge;

        if (__node_cost_pair.min_distance() < min_value) [[unlikely]] return;

        if (active(edge)) [[likely]] {

        } else {
            // insert into queue of active edges, this edge can be removed as soon as its throwaway distance is reached
            base_node_id_type src = _M_graph.base_graph().source(edge);
            base_node_id_type dest = _M_graph.base_graph().destination(edge);
            distance_type edge_length = distance(_M_graph.node(src).coordinates,
                                                 _M_graph.node(dest).coordinates);

            // ensure that aggregate is kept until no shortest paths over its nodes can be found
            _M_active_aggregates.push({edge, __node_cost_pair.value() + 10.0F * edge_length});  // use node radii

            // setup label information
            auto node_count = _M_graph.steiner_info(edge).node_count;
            _M_expanded_node_aggregates[edge] = std::make_shared<aggregate_info>(
                    std::vector<label_type>(node_count, default_value));
        }

        if (_M_expanded_node_aggregates[edge]->labels[steiner_index].distance > __node_cost_pair.distance)
            _M_expanded_node_aggregates[edge]->labels[steiner_index] = __node_cost_pair;
    }

    void label(steiner_graph::node_id_type __node, node_cost_pair_type __node_cost_pair) {
        label_preliminary(__node, __node_cost_pair);
        max_distance = std::max(max_distance, __node_cost_pair.distance);
        set_frontier_distance(__node_cost_pair.value() - frontier_width);
    };
};

