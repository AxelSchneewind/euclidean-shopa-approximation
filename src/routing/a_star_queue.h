#pragma once

#include "../graph/base_types.h"

#include "dijkstra_queues.h"


struct a_star_info {
private:
    // value from a* heuristic (distance + minimal remaining distance)
    distance_t _value;

public:
    constexpr a_star_info() : _value{optional::none_value<distance_t>} {}

    constexpr a_star_info(distance_t value) : _value{value} {}

    bool operator==(a_star_info const &) const = default;

    distance_t min_distance() const { return _value; };

    distance_t const& value() const { return _value; }

    distance_t &value() { return _value; }
};

template<>
constexpr a_star_info optional::none_value<a_star_info> = {infinity<distance_t>};

struct compare_heuristic {
public:
    compare_heuristic() = default;

    template<typename NodeCostPair>
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.value() > n2.value();
    };
};


template<typename Graph, typename NodeCostPair, typename Comp = compare_heuristic> requires requires(
        NodeCostPair n) { n.value(); }
class a_star_queue : public dijkstra_queue<Graph, NodeCostPair, Comp> {
private:
    using base_queue_type = dijkstra_queue<Graph, NodeCostPair, Comp>;
    Graph const &_graph;
    coordinate_t _target_coordinate;
    static constexpr double factor = 1.0; // must be <= 1 for correctness

public:
    using value_type = NodeCostPair;

    a_star_queue(Graph const &graph, Comp comp = Comp{})
            : dijkstra_queue<Graph, NodeCostPair, Comp>(graph, comp), _graph(graph) {}

    void push(Graph::node_id_type node, Graph::node_id_type predecessor, distance_t dist) {
        NodeCostPair ncp(node, predecessor, dist);
        ncp.value() = dist + distance(_target_coordinate, _graph.node(node).coordinates);
        base_queue_type::emplace(ncp);
    }

    void init(Graph::node_id_type /*start_node*/, Graph::node_id_type target_node) {
        while (!dijkstra_queue<Graph, NodeCostPair, Comp>::empty())
            dijkstra_queue<Graph, NodeCostPair, Comp>::pop();

        _target_coordinate = _graph.node(target_node).coordinates;
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> nodes, std::span<coordinate_t> coordinates) {
        // can be vectorized
        for (int i = 0; i < nodes.size(); ++i) {
            nodes[i].info().value() = nodes[i].distance() + factor * distance(_target_coordinate, coordinates[i]);
        }

        for (auto&& ncp: nodes)
            base_queue_type::emplace(ncp);
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> nodes) {
        static std::vector<coordinate_t> coordinates;

        // get coordinates
        coordinates.resize(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            coordinates[i] = _graph.node(nodes[i].node()).coordinates;
        }

        push_range(nodes, std::span{coordinates.begin(), coordinates.end()});
    }

    void set_target(coordinate_t coordinates) {
        _target_coordinate = coordinates;
    }
};

