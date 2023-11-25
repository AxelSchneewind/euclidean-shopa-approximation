#pragma once

#include "steiner_graph.h"
#include "../graph/graph.h"

#include "../routing/dijkstra_concepts.h"

#include <queue>


template<typename T>
concept DistanceNodeCostPair = HasDistance<T, steiner_graph::distance_type> && HasNode<T, steiner_graph::node_id_type>;

/**
 * only stores labels for relevant nodes in a graph with steiner points
 * @tparam NodeCostPair
 */
template<DistanceNodeCostPair NodeCostPair, typename Label>
class frontier_labels {
private:
    using node_id_type = steiner_graph::node_id_type;
    using base_node_id_type = steiner_graph::triangle_node_id_type;
    using base_edge_id_type = steiner_graph::triangle_edge_id_type;
    using distance_type = steiner_graph::distance_type;
    using node_cost_pair_type = NodeCostPair;

    using label_type = Label;

    struct base_edge_info {
        std::vector<label_type> labels;
    };

    steiner_graph const &_M_graph;

    std::unordered_map<base_node_id_type, std::shared_ptr<base_edge_info>> _M_edge_info;


    struct aggregate_node_info {
        base_edge_id_type edge;
        steiner_graph::distance_type throwaway_distance;
    };

    struct compare_edges {
        bool operator()(aggregate_node_info const &e1, aggregate_node_info const &e2) {
            return e1.throwaway_distance > e2.throwaway_distance;
        }
    };

    std::priority_queue<aggregate_node_info, std::vector<aggregate_node_info>, compare_edges> active_edges;

public:
    using label_type = label;

    static constexpr size_t SIZE_PER_NODE = 0;
    static constexpr size_t SIZE_PER_EDGE = sizeof(std::shared_ptr<base_edge_info>);

    frontier_labels(steiner_graph const &__graph) : _M_graph(__graph), _M_edge_info{} {};

    // init for given query
    void init(node_id_type __start_node, node_id_type __target_node) {
        _M_edge_info.clear();
    };

    // NOT CORRECT
    bool reached(node_id_type __node) const {
        return _M_edge_info.contains(__node.edge);
    }

    label_type get(node_id_type __node) const {
        if (reached(__node))
            return _M_edge_info.at(__node.edge)->labels[__node.steiner_index];
        else
            return {infinity<distance_type>};
    }

    bool active(base_edge_id_type __edge) const {
        return _M_edge_info.contains(__edge);
    };

    void label(steiner_graph::node_id_type __node, node_cost_pair_type __node_cost_pair) {
        int steiner_index = __node.steiner_index;
        base_edge_id_type edge = __node.edge;

        if (active(edge)) {
        } else {
            // insert into queue of active edges, this edge can be removed as soon as the distance of the current node plus the length of it is reached
            base_node_id_type src = _M_graph.base_graph().source(edge);
            base_node_id_type dest = _M_graph.base_graph().destination(edge);
            distance_type edge_length = distance(_M_graph.node(src).coordinates,
                                                 _M_graph.node(dest).coordinates);
            active_edges.push({edge, __node_cost_pair.distance + edge_length});

            // setup label information
            auto node_count = _M_graph.steiner_info(edge).node_count;
            _M_edge_info[edge] = std::make_shared<base_edge_info>(
                    base_edge_info{std::vector<label_type>(node_count, {infinity<distance_type>})});
        }

        _M_edge_info[edge]->labels[steiner_index] = {__node_cost_pair.distance};

        // clear edge with lowest distance if it is lower than the distance from current ncp
        while (__node_cost_pair.distance > active_edges.top().throwaway_distance) {
            base_edge_id_type edge = active_edges.top().edge;
            _M_edge_info.erase(edge);
            active_edges.pop();
        }
    };
};


// TODO move somewhere else
struct distance_label {
    using distance_type = steiner_graph::distance_type;

    distance_type distance;
    static constexpr steiner_graph::node_id_type predecessor = none_value<steiner_graph::node_id_type>;

    constexpr distance_label() : distance(infinity<distance_type>) {}

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
