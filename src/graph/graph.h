#pragma once

#include "unidirectional_adjacency_list.h"
#include "adjacency_list.h"
#include "subgraph.h"

#include "base_types.h"
#include "../routing/dijkstra_concepts.h"
#include "../util/counting_iterator.h"

#include <unordered_map>
#include <vector>


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
    using path_type = path<graph<NodeInfo, EdgeInfo, NodeId, EdgeId>>;
    using subgraph_type = subgraph<graph<NodeInfo, EdgeInfo, NodeId, EdgeId>>;

    using topology_type = adjacency_list_type;
private:

    // node data
    std::vector<NodeInfo> _M_node_list;

    // topology of _M_forward_search and backward graphs
    adjacency_list_type _M_adjacency_list;

    graph(std::vector<NodeInfo> &&__nodes, adjacency_list<NodeId, EdgeInfo> &&__list);

public:
    static constexpr size_t SIZE_PER_NODE = sizeof(NodeInfo) + adjacency_list_type::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = adjacency_list_type::SIZE_PER_EDGE;



    // move constructor
    graph(graph &&__graph) noexcept;

    // constructors
    graph(const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &__other) = default;

    graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator=(const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &) = default;

    graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator=(graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &&) = default;


    // destructor
    ~graph();

    inline std::span<const NodeInfo> nodes() const;

    counter<node_id_type> node_ids() const;

    counter<edge_id_type> edge_ids() const { return {edge_count()}; };

    inline size_t node_count() const;

    inline size_t edge_count() const;

    inline NodeInfo node(NodeId __node_id) const;
    inline NodeInfo& node(NodeId __node_id) { return _M_node_list[__node_id]; };

    EdgeInfo edge(EdgeId __edge_id) const;

    NodeId source(EdgeId __edge_id) const;

    NodeId destination(EdgeId __edge_id) const;

    inline const adjacency_list<NodeId, EdgeInfo> &topology() const;

    inline adjacency_list<NodeId, EdgeInfo> inverse_topology() const;

    EdgeId edge_id(NodeId __src, NodeId __dest) const;

    bool has_edge(NodeId __src, NodeId __dest) const;

    std::span<const internal_adjacency_list_edge<NodeId, EdgeInfo>>
    outgoing_edges(NodeId __node) const;

    std::span<const internal_adjacency_list_edge<NodeId, EdgeInfo>>
    incoming_edges(NodeId __node) const { return outgoing_edges(__node); };

    distance_type path_length(const path_type &__route) const;

    subgraph_type make_subgraph(const path_type &__route) const;

    subgraph_type make_subgraph(std::vector<NodeId> &&__nodes, std::vector<EdgeId> &&__edges) const;

    template<RoutableGraph Other, typename Subgraph>
    static graph make_graph(const Other &__base_graph, const Subgraph &__subgraph);

    static graph make_graph(std::vector<NodeInfo> &&__nodes, adjacency_list<NodeId, EdgeInfo> &&__forward);

    static graph make_graph(std::vector<NodeInfo> &&__nodes,
                            const std::shared_ptr<unidirectional_adjacency_list<NodeId, EdgeInfo>> &__forward);

};

static_assert(RoutableGraph<graph<int, int, int, int>>);


template<typename Nid>
std::ostream &
operator<<(std::ostream &__stream, path<Nid> const &__r);

