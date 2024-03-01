#pragma once

#include "../graph/base_types.h"

#include "dijkstra_concepts.h"
#include "dijkstra_queues.h"

#include <queue>
#include <vector>


template<RoutableGraph Graph, typename NodeCostPair, typename Comp>
requires requires {requires HasPredecessor<NodeCostPair> && HasDistance<NodeCostPair> && HasHeuristic<NodeCostPair>;}
class dijkstra_queue<Graph, NodeCostPair, Comp> : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp> {
private:
    using base_queue_type = std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>;
    Graph const &_graph;
    coordinate_t _target_coordinate;

    // must be <= 1 for correctness
    static constexpr double factor = 1.0;

    std::size_t _max_size{0};

public:
    using value_type = NodeCostPair;

    dijkstra_queue(Graph const &graph, Comp comp = Comp{})
            : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(comp), _graph(graph) {}

    using base_queue_type::empty;
    using base_queue_type::pop;
    using base_queue_type::top;

    std::size_t max_size() const { return _max_size; }

    void push(Graph::node_id_type node, Graph::node_id_type predecessor, distance_t dist) {
        NodeCostPair ncp{node, predecessor, dist};
        ncp.heuristic() = dist + distance(_target_coordinate, _graph.node(node).coordinates);
        base_queue_type::emplace(ncp);
    }

    void init(Graph::node_id_type /*start_node*/, Graph::node_id_type target_node) {
        base_queue_type::c.clear();
        _target_coordinate = _graph.node(target_node).coordinates;
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> nodes, std::span<coordinate_t> coordinates) {
        // can be vectorized
        for (int i = 0; i < nodes.size(); ++i) {
            nodes[i].heuristic() = nodes[i].distance() + factor * distance(_target_coordinate, coordinates[i]);
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

