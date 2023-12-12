#pragma once

#include <vector>

template<typename Graph>
struct default_neighbors {
private:
    Graph const &graph;

public:
    default_neighbors(Graph const &graph) : graph(graph) {}

    template<typename... Args>
    default_neighbors(Graph const &graph, Args const &...) : graph(graph) {}

    template<typename NodeCostPair>
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        out.clear();
        for (auto &edge: graph.outgoing_edges(node.node)) {
            out.emplace_back(edge.destination, node.node, node.distance + edge.info.cost);
        }
    }
};