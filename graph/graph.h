#pragma once

#include "adjacency_list.h"
#include "bidirectional_adjacency_list.h"
#include <unordered_map>
#include <vector>


template<typename node_info, typename edge_info>
class graph_t {
public:
    struct path_t {
        vector<node_id_t> nodes;
    };
    struct subgraph_t {
        std::vector<node_id_t> nodes;
        std::vector<edge_id_t> edges;

        subgraph_t(std::vector<node_id_t> &&n, std::vector<edge_id_t> &&e) : nodes(std::move(n)), edges(std::move(e)) {}
    };

private:
    // node data
    vector<node_info> node_list;// accessed via node_id_t

    // topology of forward and backward graphs
    bidirectional_adjacency_list<edge_info> list;

public:
    // constructors
    graph_t(const node_id_t nodeCount,
            const edge_id_t edgeCount,
            vector<node_info> &&nodes,
            bidirectional_adjacency_list<edge_info> &&forward);

    graph_t(const node_id_t nodeCount, const edge_id_t edgeCount, vector<node_info> &&nodes, const std::shared_ptr<const adjacency_list<edge_info> > &forward);

    // move constructor
    graph_t(graph_t &&graph) noexcept;

    // destructor
    ~graph_t();

    inline std::span<const node_info> nodes() const;

    [[nodiscard]] inline const size_t node_count() const;
    [[nodiscard]] inline const size_t edge_count() const;

    inline const node_info &node(const node_id_t &node_id) const;
    [[nodiscard]] inline const coordinate_t &coordinates(const node_id_t &node_id) const;

    // TODO remove
    [[deprecated]] inline const bidirectional_adjacency_list<edge_info> &get_list() const;

    inline const adjacency_list<edge_info> &forward() const { return list.forward(); };
    inline const adjacency_list<edge_info> &backward() const { return list.backward(); };

    // gets the id that a node in the forward graph has in the backward graph
    node_id_t backward_node_id(node_id_t node) const { return list._backward_node_ids->at(node); };
    // gets the id that a node in the backward graph has in the forward graph
    node_id_t forward_node_id(node_id_t node) const { return list._forward_node_ids->at(node); };

    subgraph_t make_subgraph(const path_t &route) const;

    graph_t make_graph(const subgraph_t &subgraph) const;
};

template<typename node_info, typename edge_info>
std::ostream &
operator<<(std::ostream &stream, typename graph_t<node_info, edge_info>::path_t &r);


template<typename node_info, typename edge_info>
graph_t<node_info, edge_info>::subgraph_t
graph_t<node_info, edge_info>::make_subgraph(const graph_t::path_t &route) const {
    std::vector<node_id_t> nodes;
    std::vector<edge_id_t> edges;

    for (int i = 0; i < route.nodes.size() - 1; i++) {
        nodes.push_back(route.nodes[i]);
        edges.push_back(list.forward().edge_index(route.nodes[i], route.nodes[i + i]));
    }

    return graph_t::subgraph_t(std::move(nodes), std::move(edges));
}


template<typename N, typename E>
graph_t<N, E>
graph_t<N, E>::make_graph(const graph_t<N, E>::subgraph_t &subgraph) const {
    node_id_t node_count = subgraph.nodes.size();
    edge_id_t edge_count = subgraph.edges.size();

    vector<N> nodes;
    vector<node_id_t> old_node_ids;
    std::unordered_map<node_id_t, node_id_t> new_node_ids;

    // make node list
    for (node_id_t i = 0; i < node_count; i++) {
        auto id = subgraph.nodes[i];

        nodes.push_back(node(id));
        old_node_ids.push_back(id);
        new_node_ids[id] = i;
    }

    // make adjacency list
    typename adjacency_list<E>::adjacency_list_builder forward_builder;
    forward_builder.add_node(node_count - 1);
    for (edge_id_t e: subgraph.edges) {
        node_id_t src = list.forward().source(e);
        node_id_t dest = list.forward().destination(e);
        E info = list.forward().edge(e);

        forward_builder.add_edge(src, dest, info);
    }
    auto adj_list = forward_builder.get();
    auto l = bidirectional_adjacency_list<E>(std::move(adj_list));
    return graph_t(node_count,
                   edge_count,
                   std::move(nodes),
                   std::move(l));
};


template<typename node_info, typename edge_info>
const size_t
graph_t<node_info, edge_info>::edge_count() const { return forward().edge_count(); }

template<typename node_info, typename edge_info>
const size_t
graph_t<node_info, edge_info>::node_count() const { return node_list.size(); }


template<typename node_info, typename edge_info>
const node_info &
graph_t<node_info, edge_info>::node(const node_id_t &node_id) const { return node_list[node_id]; }

template<typename node_info, typename edge_info>
std::span<const node_info, std::dynamic_extent>
graph_t<node_info, edge_info>::nodes() const { return std::span(node_list.begin(), node_list.end()); }

template<typename node_info, typename edge_info>
const coordinate_t &
graph_t<node_info, edge_info>::coordinates(const node_id_t &node_id) const { return node(node_id).coordinates; }


template<typename node_info, typename edge_info>
const bidirectional_adjacency_list<edge_info> &
graph_t<node_info, edge_info>::get_list() const {
    return list;
}


template<typename node_info, typename edge_info>
graph_t<node_info, edge_info>::~graph_t() {
    node_list.clear();
    node_list.shrink_to_fit();
}

template<typename node_info, typename edge_info>
graph_t<node_info, edge_info>::graph_t(graph_t &&graph) noexcept
    : node_list(std::move(graph.node_list)),
      list(std::move(graph.list)) {}

template<typename node_info, typename edge_info>
graph_t<node_info, edge_info>::graph_t(const node_id_t nodeCount, const edge_id_t edgeCount, vector<node_info> &&nodes, const std::shared_ptr<const adjacency_list<edge_info> > &forward)
    : node_list(std::move(nodes)),
      list(bidirectional_adjacency_list<edge_info>::make_bidirectional(forward)) {}

template<typename node_info, typename edge_info>
graph_t<node_info, edge_info>::graph_t(const node_id_t nodeCount, const edge_id_t edgeCount, vector<node_info> &&nodes, bidirectional_adjacency_list<edge_info> &&list)
    : node_list(std::move(nodes)),
      list(std::move(list)) {}
