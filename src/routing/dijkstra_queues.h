#pragma once

#include "dijkstra_concepts.h"

#include "../graph/unidirectional_adjacency_list.h"

#include "../triangulation/compact_node_info_container.h"

#include <concepts>
#include <tuple>
#include <queue>
#include <map>

// TODO remove
template<RoutableGraph Graph>
struct use_all_edges {
public:
    use_all_edges(Graph const &g) {}

    use_all_edges() = default;

    use_all_edges(use_all_edges &&) = default;

    constexpr bool operator()(Graph::node_id_type /*node*/,
                              internal_adjacency_list_edge<typename Graph::node_id_type, typename Graph::edge_info_type> /*via*/) {
        return true;
    };
};

template<RoutableGraph Graph>
struct use_upward_edges {
protected:
    Graph const &g;

public:
    use_upward_edges(Graph const &g) : g(g) {}

    use_upward_edges(use_upward_edges &&) = default;

    constexpr bool operator()(Graph::node_id_type node,
                              internal_adjacency_list_edge<typename Graph::node_id_type, typename Graph::edge_info_type> via) {
        return g.node(node).level <= g.node(via.destination).level;
    };
};

struct Default {
public:
    Default() = default;

    template<typename NodeCostPair>
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.distance() > n2.distance();
    };
};

template<RoutableGraph Graph, typename NodeCostPair, typename Comp = Default>
class dijkstra_queue : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp> {
private:
    std::size_t _max_size{0};

protected:
    using base_queue_type = std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>;

public:
    using value_type = NodeCostPair;

    dijkstra_queue(Graph const &graph, Comp comp = Comp{})
            : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(comp) {}

    void init(Graph::node_id_type start_node, Graph::node_id_type target_node) {
        while (!empty())
            pop();

        _max_size = 0;
    };

    void push(base_queue_type::value_type const& ncp) {
        assert(ncp.distance() != infinity<decltype(ncp.distance())>);
        base_queue_type::emplace(ncp);
    }

    void push(Graph::node_id_type node, Graph::node_id_type predecessor, distance_t dist) {
        NodeCostPair ncp(node, predecessor, dist);
        push(ncp);
    }

    void push_range(std::span<NodeCostPair, std::dynamic_extent> nodes) {
        for (auto&& ncp: nodes)
            push(ncp);
    }

    bool empty() const { return base_queue_type::empty(); }

    void pop() {
        _max_size = std::max(_max_size, base_queue_type::size());
        return base_queue_type::pop();
    }

    NodeCostPair top() const {
        return base_queue_type::top();
    }

    size_t max_size() const { return _max_size; }
};