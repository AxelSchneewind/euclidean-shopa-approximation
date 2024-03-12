#pragma once

#include <vector>

template<typename Graph>
struct default_neighbors {
private:
    std::shared_ptr<const Graph> graph;

public:
    default_neighbors(std::shared_ptr<Graph> graph) : graph(std::move(graph)) {}

    template<typename... Args>
    default_neighbors(std::shared_ptr<Graph> graph, Args const &...) : graph(std::move(graph)) {}

    template<typename NodeCostPair>
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        out.clear();
        for (auto&& edge: graph->outgoing_edges(node.node())) {
            out.emplace_back(edge.destination, node.node(), node.distance() + edge.info.cost);
        }
    }
};