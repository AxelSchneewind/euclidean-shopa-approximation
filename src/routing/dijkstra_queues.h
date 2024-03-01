#pragma once

#include "dijkstra_concepts.h"

#include <cstddef>
#include <queue>
#include <vector>

struct compare_distance {
public:
    compare_distance() = default;

    template<HasDistance NodeCostPair>
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.distance() > n2.distance();
    };
};

struct compare_heuristic {
public:
    compare_heuristic() = default;

    template<typename NodeCostPair>
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.value() > n2.value();
    };
};


template<RoutableGraph Graph, typename NodeCostPair, typename Comp = compare_distance>
class dijkstra_queue{};

template<RoutableGraph Graph, typename NodeCostPair, typename Comp> requires
    requires {requires HasPredecessor<NodeCostPair> && HasDistance<NodeCostPair> && !HasHeuristic<NodeCostPair>;}
class dijkstra_queue<Graph, NodeCostPair, Comp> : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp> {
private:
    std::size_t _max_size{0};

protected:
    using base_queue_type = std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>;

public:
    using value_type = NodeCostPair;

    dijkstra_queue(Graph const &graph, Comp comp = Comp{})
            : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(comp) {}

    void init(Graph::node_id_type start_node, Graph::node_id_type target_node) {
        base_queue_type::c.clear();

        _max_size = 0;
    };

    void push(base_queue_type::value_type&& ncp) {
        assert(ncp.distance() != infinity<decltype(ncp.distance())>);
        base_queue_type::emplace(ncp);
        _max_size = std::max(_max_size, base_queue_type::size());
    }

    template<typename... Args>
    void push(Args... args) {
        NodeCostPair ncp(args...);
        base_queue_type::emplace(ncp);
        _max_size = std::max(_max_size, base_queue_type::size());
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> nodes) {
        for (auto&& ncp: nodes)
            push(ncp);
    }

    using base_queue_type::empty;
    using base_queue_type::pop;
    using base_queue_type::top;

    size_t max_size() const { return _max_size; }
};