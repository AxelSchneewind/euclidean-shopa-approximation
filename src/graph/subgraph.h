#pragma once

#include <vector>

#include "../util/remove_duplicates.h"

template<typename BaseGraph>
struct path {
    using node_id_type = typename BaseGraph::node_id_type;
    std::vector<node_id_type> nodes;

    path() = default;

    path(path &other) = default;
    path &operator=(path const&other) = default;

    path(std::vector<node_id_type> &&n) : nodes{n} {};

    void invert() {
        for (size_t i = 0; i < nodes.size() / 2; ++i) {
            std::swap(nodes[i], nodes[nodes.size() - 1 - i]);
        }
    }

    static path concat(path const& first, path const& other) {
        auto nodes = first.nodes;

        for (size_t i = 0; i < other.nodes.size(); ++i) {
            nodes.emplace_back(other.nodes[i]);
        }

        return {std::move(nodes)};
    }
};

template<typename BaseGraph>
struct subgraph {
    using node_id_type = typename BaseGraph::node_id_type;
    using edge_id_type = typename BaseGraph::edge_id_type;

// private:
    std::vector<node_id_type> nodes;
    std::vector<edge_id_type> edges;
public:
    subgraph() = default;

    subgraph(std::vector<node_id_type>&& n, std::vector<edge_id_type>&& e) : nodes(std::move(n)), edges(std::move(e)) {};

    subgraph(subgraph&& other) noexcept : nodes(std::move(other.nodes)), edges(std::move(other.edges)) {};
    subgraph(subgraph const& other) : nodes(other.nodes), edges(other.edges) {};

    subgraph& operator=(subgraph const& other) { nodes = other.nodes; edges = other.edges; return *this; };
    subgraph& operator=(subgraph && other)  noexcept { nodes = std::move(other.nodes); edges = std::move(other.edges); return *this; };

    size_t node_count() const { return nodes.size(); }

    size_t edge_count() const { return edges.size(); }
};

template<typename Graph, std::predicate<typename Graph::node_id_type> NodePredicate>
subgraph<Graph>
filter_nodes(const subgraph<Graph> &other, NodePredicate &&predicate = NodePredicate{}) {
    std::vector<typename subgraph<Graph>::node_id_type> filtered_nodes;
    std::vector<typename subgraph<Graph>::edge_id_type> filtered_edges(other.edges);

    for (auto node_id: other.nodes)
        if (predicate(node_id))
            filtered_nodes.push_back(node_id);

    return {std::move(filtered_nodes), std::move(filtered_edges)};
};

template<typename Graph, std::predicate<typename Graph::edge_id_type> NodePredicate>
subgraph<Graph>
filter_edges(const subgraph<Graph> &other, NodePredicate &&predicate = NodePredicate{}) {
    std::vector<typename subgraph<Graph>::node_id_type> filtered_nodes(other.nodes);
    std::vector<typename subgraph<Graph>::edge_id_type> filtered_edges;

    for (auto edge_id: other.edges)
        if (predicate(edge_id))
            filtered_edges.push_back(edge_id);

    return {std::move(filtered_nodes), std::move(filtered_edges)};
};

template<typename Graph>
subgraph<Graph>
subgraphs_union(const subgraph<Graph> &first, const subgraph<Graph> &second) {
    auto nodes = first.nodes;
    auto edges = first.edges;

    for (auto&& node : second.nodes)
        nodes.emplace_back(node);
    for (auto&& edge : second.edges)
        edges.emplace_back(edge);

    remove_duplicates(nodes);
    remove_duplicates(edges);

    return {std::move(nodes), std::move(edges)};
}
