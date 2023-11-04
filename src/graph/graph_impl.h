#pragma once

#include "graph.h"

#include "base_types.h"

#include <unordered_map>
#include <vector>

template<typename NodeId, typename EdgeId>
subgraph<NodeId, EdgeId>::subgraph(std::vector<NodeId> &&__n, std::vector<EdgeId> &&__e)
        : nodes(std::move(__n)), edges(std::move(__e)) {}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_subgraph(std::vector<NodeId> &&__nodes,
                                                         std::vector<EdgeId> &&__edges) const {
    return {std::move(__nodes), std::move(__edges)};
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_subgraph(
        const graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::path &__route) const {
    std::vector<NodeId> nodes;
    std::vector<EdgeId> edges;

    if (!__route.nodes.empty()) {
        nodes.push_back(__route.nodes[0]);
    }
    for (size_t i = 0; i + 1 < __route.nodes.size(); i++) {
        auto id_current = __route.nodes[i];
        auto id_next = __route.nodes[i + 1];
        assert (id_current < node_count());
        assert (id_next < node_count());

        nodes.push_back(id_next);
        edges.push_back(_M_adjacency_list.edge_id(id_current, id_next));
    }

    return {std::move(nodes), std::move(edges)};
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph(
        const graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::subgraph &__subgraph) const {
    NodeId node_count = __subgraph.nodes.size();
    EdgeId edge_count = __subgraph.edges.size();

    std::vector<NodeInfo> nodes;
    std::vector<NodeId> old_node_ids;
    std::unordered_map<NodeId, NodeId> new_node_ids;

    // make node list
    for (node_id_t i = 0; i < node_count; i++) {
        auto node_id = __subgraph.nodes[i];

        nodes.push_back(node(node_id));
        old_node_ids.push_back(node_id);
        new_node_ids[node_id] = i;
    }

    // make one-directional adjacency list
    typename unidirectional_adjacency_list<NodeId, EdgeInfo>::adjacency_list_builder forward_builder;
    forward_builder.add_node(node_count - 1);
    for (auto edge: __subgraph.edges) {
        auto src = _M_adjacency_list.source(edge);
        auto dest = _M_adjacency_list.destination(edge);
        EdgeInfo info = _M_adjacency_list.edge(edge);

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
const graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_info_type &
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node(
        const graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_id_type &__node_id) const {
    return _M_node_list[__node_id];
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
std::span<const NodeInfo, std::dynamic_extent>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::nodes() const {
    return std::span(_M_node_list.begin(), _M_node_list.end());
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
counter<NodeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::node_ids() const {
    return counter((NodeId) node_count());
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::~graph() {
    _M_node_list.clear();
    _M_node_list.shrink_to_fit();
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::graph(graph &&__graph) noexcept
        : _M_node_list(std::move(__graph._M_node_list)), _M_adjacency_list(std::move(__graph._M_adjacency_list)) {}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::graph(std::vector<NodeInfo> &&__nodes,
                                                 adjacency_list<NodeId, EdgeInfo> &&__list)
        : _M_node_list(std::move(__nodes)), _M_adjacency_list(std::move(__list)) {}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph(std::vector<NodeInfo> &&__nodes,
                                                      adjacency_list<NodeId, EdgeInfo> &&__forward) {
    return graph(std::move(__nodes), std::move(__forward));
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::make_graph(
        std::vector<NodeInfo> &&__nodes,
        const std::shared_ptr<unidirectional_adjacency_list<NodeId, EdgeInfo>> &__forward) {
    auto adj_list = adjacency_list<NodeId, EdgeInfo>::make_bidirectional(__forward);
    return {std::move(__nodes), std::move(adj_list)};
}

template<typename NodeId>
std::ostream &
operator<<(std::ostream &__stream, path<NodeId> &__r) {
    __stream << "{ ";

    int length = __r.nodes.size();
    if (length > 0) {
        for (int i = 0; i < length - 1; i++) {
            __stream << __r.nodes[i] << ", ";
        }
        __stream << __r.nodes[length - 1];
    }

    return __stream << " }";
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
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::path_length(const path &__route) const {
    if (__route.nodes.empty()) {
        return DISTANCE_INF;
    }

    distance_t result = 0;

    for (int i = 0; i < __route.nodes.size() - 1; ++i) {
        auto from = __route.nodes[i];
        auto to = __route.nodes[i + 1];
        result += topology().edge(topology().edge_id(from, to)).cost;
    }

    return result;
}


template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
const NodeId &graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::destination(const EdgeId &__edge_id) const {
    return _M_adjacency_list.destination(__edge_id);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
const NodeId &graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::source(const EdgeId &__edge_id) const {
    return _M_adjacency_list.source(__edge_id);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
const EdgeInfo &graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::edge(const EdgeId &__edge_id) const {
    return _M_adjacency_list.edge(__edge_id);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
EdgeId graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::edge_id(const node_id_type &__src, const node_id_type &__dest) const {
    return _M_adjacency_list.edge_id(__src, __dest);
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
bool graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::has_edge(const node_id_type &__src, const node_id_type &__dest) const {
    return edge_id(__src, __dest) != NO_EDGE_ID;
}

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
std::span<const internal_adjacency_list_edge<NodeId, EdgeInfo>>
graph<NodeInfo, EdgeInfo, NodeId, EdgeId>::outgoing_edges(const node_id_type &__node) const {
    return topology().outgoing_edges(__node);
}
