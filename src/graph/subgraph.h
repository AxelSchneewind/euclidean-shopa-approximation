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
    BaseGraph const &base;
    std::vector<node_id_type> nodes;
    std::vector<edge_id_type> edges;
public:

    size_t node_count() const { return nodes.size(); }

    size_t edge_count() const { return edges.size(); }

    subgraph(subgraph const &other) = default;

    subgraph &operator=(subgraph const &other) = default;

    subgraph(BaseGraph const &base_graph) : base(base_graph) {};

    subgraph(BaseGraph const &base_graph, std::vector<node_id_type> &&__n, std::vector<edge_id_type> &&__e);
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

