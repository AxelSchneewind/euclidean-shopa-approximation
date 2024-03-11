#pragma once

#include "dijkstra_concepts.h"

#include <cstddef>
#include <queue>
#include <vector>

struct compare_distance {
public:
    compare_distance() = default;

    template<HasDistance NodeCostPair>
    [[using gnu : always_inline, hot]]
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.distance() > n2.distance();
    };
};

struct compare_heuristic {
public:
    compare_heuristic() = default;

    template<typename NodeCostPair>
    [[using gnu : always_inline, hot]]
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return n1.heuristic() > n2.heuristic();
    };
};

template<typename Labels>
struct compare_heuristic_remote {
private:
    std::shared_ptr<Labels> _labels;
public:
    template<typename Graph>
    compare_heuristic_remote(Graph&&, std::shared_ptr<Labels> labels) : _labels{std::move(labels)} {};

    template<typename NodeCostPair>
    [[using gnu : always_inline, hot]]
    constexpr bool operator()(const NodeCostPair &n1, const NodeCostPair &n2) {
        return _labels->at(n1.node()).heuristic() > _labels->at(n2.node()).heuristic();
    };
};

struct no_heuristic {
    template <typename Graph, typename Labels>
    no_heuristic(Graph&&, Labels&&) {};

    template<typename R>
    void operator()(R& node) {
        if constexpr(HasHeuristic<std::remove_reference_t<R>>) {
            node.heuristic() = node.distance();
        }
    }
};

template<typename Graph>
struct a_star_heuristic {
private:
    using node_id_type    = typename Graph::node_id_type;
    using distance_type   = typename Graph::distance_type;
    using coordinate_type = typename Graph::coordinate_type;

    std::shared_ptr<Graph const> _graph;
    coordinate_type _target_coordinate{};

    // static constexpr typename Graph::distance_type factor = 1.0;

public:
    constexpr a_star_heuristic(std::shared_ptr<Graph> graph) : _graph{graph} {}

    template<typename Labels>
    constexpr a_star_heuristic(std::shared_ptr<Graph> graph, Labels&&) : _graph{graph} {}

    void init(node_id_type /*source*/, node_id_type target) {
        _target_coordinate = _graph->node_coordinates(target);
    }

    template<typename R>
    void operator()(R& node) {
        typename Graph::coordinate_type coordinate = _graph->node_coordinates(node.node());
        operator()(node, coordinate);
    }

    template<typename R, typename C>
    void operator()(R& node, C const& coordinate) {
        if constexpr(HasDistance<typename std::remove_reference_t<R>>) {
            node.heuristic() = node.distance() + /*factor */ distance(coordinate, _target_coordinate);
        } else {
            node.heuristic() += distance(coordinate, _target_coordinate);
        }
        assert(node.heuristic() >= node.distance());
    }
};


template<typename NodeCostPair, typename Comp = compare_distance>
class dijkstra_queue : protected std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp> {
protected:
    using base_queue_type = std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>;

public:
    using value_type = NodeCostPair;

    template <typename Graph, typename Labels>
    dijkstra_queue(std::shared_ptr<Graph> graph, std::shared_ptr<Labels> labels) requires requires { Comp(graph, labels); }
        : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(Comp(graph, labels)) {}

    template <typename Graph, typename Labels>
    dijkstra_queue(std::shared_ptr<Graph> /*graph*/, std::shared_ptr<Labels> /*labels*/)
        : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(Comp{}) {}

    dijkstra_queue(Comp comp = {})
        : std::priority_queue<NodeCostPair, std::vector<NodeCostPair>, Comp>(comp) {}

    template <typename NodeId>
    void init(NodeId /*start_node*/, NodeId /*target_node*/) {
        base_queue_type::c.clear();
    };

    template<typename... Args>
    void push(Args... args) {
        base_queue_type::emplace(args...);
    }

    void push(base_queue_type::value_type&& ncp) {
        assert(ncp.distance() != infinity<decltype(ncp.distance())>);
        base_queue_type::emplace(std::move(ncp));
    }


    template <typename R>
    void push_range(R&& nodes) {
        for (auto&& ncp: nodes)
            push(ncp);
    }

    using base_queue_type::empty;
    using base_queue_type::pop;
    using base_queue_type::top;
    using base_queue_type::size;

    void clear() {
        base_queue_type::c.clear();
    }
};