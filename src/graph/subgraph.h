#pragma once

#include <vector>

template<typename BaseGraph>
struct path {
    using node_id_type = typename BaseGraph::node_id_type;
    BaseGraph const &base;
    std::vector<node_id_type> nodes;

    path(path &other) = default;
    path &operator=(path &other) = default;

    path(BaseGraph const &base_graph, std::vector<node_id_type> &&__n) : base{base_graph}, nodes{__n} {};
};

template<typename BaseGraph>
struct subgraph {
    using node_id_type = typename BaseGraph::node_id_type;
    using edge_id_type = typename BaseGraph::edge_id_type;

// private:
    const BaseGraph &base;
    std::vector<node_id_type> nodes;
    std::vector<edge_id_type> edges;
public:
    subgraph(BaseGraph const& base, std::vector<node_id_type>&& n, std::vector<edge_id_type>&& e) : base{base}, nodes(std::move(n)), edges(std::move(e)) {};
    subgraph(BaseGraph const& base) : base{base} {};

    subgraph(subgraph&& other) noexcept : base{other.base}, nodes(std::move(other.nodes)), edges(std::move(other.edges)) {};
    subgraph(subgraph const& other) : base{other.base}, nodes(other.nodes), edges(other.edges) {};

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

