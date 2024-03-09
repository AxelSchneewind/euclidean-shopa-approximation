#pragma once

#include "graph.h"

#include "base_types.h"

#include <span>
#include <unordered_map>
#include <vector>

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph_type
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_subgraph(std::vector<NodeId> &&nodes,
                                                         std::vector<EdgeId> &&edges) const {
    return {std::move(nodes), std::move(edges)};
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph_type
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_subgraph(
        const graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::path_type &route) const {
    std::vector<NodeId> nodes;
    std::vector<EdgeId> edges;

    if (!route.nodes.empty()) {
        nodes.push_back(route.nodes[0]);
    }
    for (size_t i = 0; i + 1 < route.nodes.size(); i++) {
        auto id_current = route.nodes[i];
        auto id_next = route.nodes[i + 1];
        assert (id_current < node_count());
        assert (id_next < node_count());

        nodes.push_back(id_next);
        edges.push_back(_M_adjacency_list.edge_id(id_current, id_next));
    }

    return {std::move(nodes), std::move(edges)};
}


template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
template<RoutableGraph Other, typename Subgraph>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph(const Other & base_graph,
                                                      const Subgraph& subgraph) {
    size_t node_count = subgraph.node_count();

    std::vector<NodeInfo> nodes;
    std::unordered_map<typename Other::node_id_type, NodeId> new_node_ids;

    // make node list and store indices for each node
    for (size_t i = 0; i < node_count; i++) {
        auto node_id = subgraph.nodes[i];

        nodes.emplace_back(base_graph.node(node_id));
        new_node_ids[node_id] = (NodeId) i;
    }

    // make one-directional adjacency list
    typename adjacency_list<NodeId, EdgeInfo>::builder forward_builder(node_count);
    for (auto&& edge: subgraph.edges) {
        if (optional::is_none(edge)) continue;

        auto&& src = base_graph.source(edge);
        auto&& dest = base_graph.destination(edge);
        assert(!optional::is_none(src) && !optional::is_none(dest));
        if((!base_graph.has_edge(src, dest) && !base_graph.has_edge(dest, src)) || !new_node_ids.contains(src) || !new_node_ids.contains(dest)) continue;

        auto&& info = base_graph.edge(edge);

        forward_builder.add_edge(new_node_ids[src], new_node_ids[dest], info);
    }

    // make bidirectional adjacency list
    auto list = adjacency_list<NodeId, EdgeInfo>::make_bidirectional(forward_builder.get());

    return graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph(std::move(nodes), std::move(list));
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
size_t
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::edge_count() const {
    return topology().edge_count();
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
size_t
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_count() const {
    return _M_node_list.size();
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_info_type
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node(
        graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_id_type node_id) const {
    return _M_node_list[node_id];
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
std::span<NodeInfo, std::dynamic_extent>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::nodes() const {
    return std::span(_M_node_list.begin(), _M_node_list.end());
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
counter<NodeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_ids() const {
    return _M_adjacency_list.node_ids();
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::~graph() {
    _M_node_list.clear();
    _M_node_list.shrink_to_fit();
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::graph(graph &&graph) noexcept
        : _M_node_list(std::move(graph._M_node_list)), _M_adjacency_list(std::move(graph._M_adjacency_list)) {}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::graph(std::vector<NodeInfo> &&nodes,
                                                 adjacency_list<NodeId, EdgeInfo> &&list)
        : _M_node_list(std::move(nodes)), _M_adjacency_list(std::move(list)) {}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph(std::vector<NodeInfo> &&nodes,
                                                      adjacency_list<NodeId, EdgeInfo> &&forward) {
    return graph(std::move(nodes), std::move(forward));
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph(
        std::vector<NodeInfo> &&nodes,
        const std::shared_ptr<unidirectional_adjacency_list<NodeId, EdgeInfo>> &forward) {
    auto adj_list = adjacency_list<NodeId, EdgeInfo>::make_bidirectional(forward);
    return {std::move(nodes), std::move(adj_list)};
}

template<typename Graph>
std::ostream &
operator<<(std::ostream &stream, path<Graph> const &r) {
    stream << "{ ";

    int length = r.nodes.size();
    if (length > 0) {
        for (int i = 0; i < length - 1; i++) {
            stream << r.nodes[i] << ", ";
        }
        stream << r.nodes[length - 1];
    }

    return stream << " }";
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
adjacency_list<NodeId, EdgeInfo>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::inverse_topology() const {
    return graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::adjacency_list_type::invert(_M_adjacency_list);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
const adjacency_list<NodeId, EdgeInfo> &
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::topology() const {
    return _M_adjacency_list;
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::distance_type
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::path_length(const path_type &route) const {
    if (route.nodes.empty()) {
        return infinity<distance_type>;
    }

    distance_t result = 0;
    for (int i = 0; i < route.nodes.size() - 1; ++i) {
        auto from = route.nodes[i];
        auto to = route.nodes[i + 1];
        result += topology().edge(topology().edge_id(from, to)).cost;
    }

    return result;
}


template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
NodeId graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::destination(EdgeId edge_id) const {
    return _M_adjacency_list.destination(edge_id);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
NodeId graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::source(EdgeId edge_id) const {
    return _M_adjacency_list.source(edge_id);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
EdgeInfo graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::edge(EdgeId edge_id) const {
    return _M_adjacency_list.edge(edge_id);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
EdgeId graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::edge_id(node_id_type src, node_id_type dest) const {
    return _M_adjacency_list.edge_id(src, dest);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
bool graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::has_edge(node_id_type src, node_id_type dest) const {
    return !optional::is_none(edge_id(src, dest));
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
std::span<const internal_adjacency_list_edge<NodeId, EdgeInfo>>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::outgoing_edges(node_id_type node) const {
    return topology().outgoing_edges(node);
}
