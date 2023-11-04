#pragma once

#include "base_types.h"
#include "unidirectional_adjacency_list.h"
#include "adjacency_list.h"
#include "../routing/dijkstra_concepts.h"
#include <unordered_map>
#include <vector>
#include "../util/counting_iterator.h"

template<typename NodeId>
struct path {
    std::vector<NodeId> nodes;
};

template<typename NodeId, typename EdgeId>
struct subgraph {
    std::vector<NodeId> nodes;
    std::vector<EdgeId> edges;

    subgraph() = default;

    subgraph(std::vector<NodeId> &&__n, std::vector<EdgeId> &&__e);
};

/**
 * stores a directed graph consisting of nodes and edges
 * @tparam NodeInfo the type of information to store for each node
 * @tparam EdgeInfo the type of information to store for each edge
 * @tparam NodeId the type by which nodes can be accessed
 * @tparam EdgeId the type by which edges can be accessed
 */
template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
class graph {
public:
    using node_id_type = NodeId;
    using edge_id_type = EdgeId;
    using node_info_type = NodeInfo;
    using distance_type = distance_t;
    using edge_info_type = EdgeInfo;
    using adjacency_list_type = adjacency_list<NodeId, EdgeInfo>;
    using path = path<NodeId>;
    using subgraph = subgraph<NodeId, EdgeId>;

    using topology_type = adjacency_list_type;
private:

// node data
    std::vector<NodeInfo> _M_node_list;

    // topology of forward and backward graphs
    adjacency_list_type _M_adjacency_list;

    graph(std::vector<NodeInfo> &&__nodes, adjacency_list<NodeId, EdgeInfo> &&__list);

public:
    // move constructor
    graph(graph &&__graph) noexcept;

    // constructors
    graph(const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &__other) = default;

    graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator=(const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &) = default;

    graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator=(graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &&) = default;


    // destructor
    ~graph();

    inline std::span<const NodeInfo> nodes() const;

    inline counter<NodeId> node_ids() const;

    inline size_t node_count() const;

    inline size_t edge_count() const;

    inline const NodeInfo &node(const NodeId &__node_id) const;

    const EdgeInfo &edge(const EdgeId &__edge_id) const;

    const NodeId &source(const EdgeId &__edge_id) const;

    const NodeId &destination(const EdgeId &__edge_id) const;

    inline const adjacency_list<NodeId, EdgeInfo> &topology() const;

    inline adjacency_list<NodeId, EdgeInfo> inverse_topology() const;

    EdgeId edge_id(const NodeId &__src, const NodeId &__dest) const;

    bool has_edge(const NodeId &__src, const NodeId &__dest) const;

    std::span<const internal_adjacency_list_edge<NodeId, EdgeInfo>>
    outgoing_edges(const NodeId &__node) const;

    distance_type path_length(const path &__route) const;

    subgraph make_subgraph(const path &__route) const;

    subgraph make_subgraph(std::vector<NodeId> &&__nodes, std::vector<EdgeId> &&__edges) const;

    graph make_graph(const subgraph &__subgraph) const;

    static graph make_graph(std::vector<NodeInfo> &&__nodes, adjacency_list<NodeId, EdgeInfo> &&__forward);

    static graph make_graph(std::vector<NodeInfo> &&__nodes,
                            const std::shared_ptr<unidirectional_adjacency_list<NodeId, EdgeInfo>> &__forward);

};


static_assert(RoutableGraph<graph<int, int, int, int>>);


template<typename Nid>
std::ostream &
operator<<(std::ostream &__stream, path<Nid> &__r);

