#pragma once

#include "../graph/base_types.h"

#include "dijkstra_queues.h"


struct a_star_info {
    // value from a* heuristic (distance + minimal remaining distance)
    distance_t _value;

    constexpr a_star_info() : _value{none_value<distance_t>} {}

    constexpr a_star_info(distance_t _value) : _value{_value} {}

    bool operator==(a_star_info const &) const = default;

    distance_t min_distance() const { return _value; };

    distance_t value() const { return _value; }

    distance_t &value() { return _value; }
};

template<>
constexpr a_star_info none_value<a_star_info> = {infinity<distance_t>};


struct A_Star {
public:
    A_Star() {};

    template<typename NodeCostPair>
    constexpr bool operator()(const NodeCostPair &__n1, const NodeCostPair &__n2) {
        return __n1.value() > __n2.value();
    };
};


template<typename Graph, typename NodeCostPair, typename Comp = A_Star> requires requires(
        NodeCostPair n) { n.value(); }
class a_star_queue : public dijkstra_queue<Graph, NodeCostPair, Comp> {
private:
    using base_queue_type = dijkstra_queue<Graph, NodeCostPair, Comp>;
    Graph const &_M_graph;
    coordinate_t _M_target_coordinates;
    static constexpr double factor = 1.0; // must be <= 1 for correctness
    float additional_distance;

public:
    using value_type = NodeCostPair;

    a_star_queue(Graph const &__graph, Comp __comp = Comp{})
            : dijkstra_queue<Graph, NodeCostPair, Comp>(__graph, __comp), _M_graph(__graph) {}

    void push(Graph::node_id_type __node, Graph::node_id_type __predecessor, distance_t __dist) {
        NodeCostPair ncp(__node, __predecessor, __dist,
                         {__dist + distance(_M_target_coordinates, _M_graph.node(__node).coordinates)});
        base_queue_type::push(ncp);
    }

    void init(Graph::node_id_type /*__start_node*/, Graph::node_id_type __target_node) {
        while (!dijkstra_queue<Graph, NodeCostPair, Comp>::empty())
            dijkstra_queue<Graph, NodeCostPair, Comp>::pop();

        _M_target_coordinates = _M_graph.node(__target_node).coordinates;
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> __nodes) {
        static std::vector<coordinate_t> coordinates;

        // get coordinates
        coordinates.resize(__nodes.size());
        for (int i = 0; i < __nodes.size(); ++i) {
            coordinates[i] = _M_graph.node(__nodes[i].node).coordinates;
        }

        // can be vectorized
        for (int i = 0; i < __nodes.size(); ++i) {
            __nodes[i].info.value() =
                    __nodes[i].distance + factor * distance(_M_target_coordinates, coordinates[i]);
        }

        for (auto ncp: __nodes)
            base_queue_type::push(ncp);
    }

    void set_target(coordinate_t coordinates, float additional) {
        _M_target_coordinates = coordinates;
        additional_distance = additional;
    }
};

