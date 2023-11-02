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

template<typename NodeInfo, typename EdgeInfo, typename NodeId, typename EdgeId>
class graph {
public:
    using node_id_type = NodeId;
    using edge_id_type = EdgeId;
    using node_info_type = NodeInfo;
    using distance_type = distance_t;
    using edge_info_type = EdgeInfo;
    using adjacency_list_type = adjacency_list<EdgeInfo>;
    using path = path<NodeId>;
    using subgraph = subgraph<NodeId, EdgeId>;

private:
    // node data
    std::vector<NodeInfo> _M_node_list; // accessed via node_id_t

    // topology of topology and backward graphs
    adjacency_list<EdgeInfo> _M_adjacency_list;

    // constructors
    graph(std::vector<NodeInfo> &&__nodes, adjacency_list<EdgeInfo> &&__list);

    graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator=(const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &) = default;

    graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &operator=(graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &&) = default;

public:
    // move constructor
    graph(graph &&__graph) noexcept;

    graph(const graph<NodeInfo, EdgeInfo, NodeId, EdgeId> &other) = default;

    // destructor
    ~graph();

    static graph make_graph(std::vector<NodeInfo> &&__nodes, adjacency_list<EdgeInfo> &&__forward);

    static graph make_graph(std::vector<NodeInfo> &&__nodes,
                            const std::shared_ptr<unidirectional_adjacency_list<node_id_type, EdgeInfo>> &__forward);

    inline std::span<const node_info_type> nodes() const;
    inline counter<node_id_type> node_ids() const;

    inline size_t node_count() const;

    inline size_t edge_count() const;

    inline const node_info_type &node(const node_id_type &__node_id) const;

    const edge_info_type &edge(const edge_id_type &__edge_id) const { return _M_adjacency_list.edge(__edge_id); }

    const node_id_type &source(const edge_id_type &__edge_id) const { return _M_adjacency_list.source(__edge_id); }

    const node_id_type &destination(const edge_id_type &__edge_id) const {
        return _M_adjacency_list.destination(__edge_id);
    }

    [[deprecated]]
    const adjacency_list<edge_info_type> &list() const { return _M_adjacency_list; };

    [[deprecated]]
    inline const adjacency_list<edge_info_type> &topology() const;

    [[deprecated]]
    inline const adjacency_list<edge_info_type> &inverse_topology() const;

    edge_id_type edge_id(const node_id_type &src, const node_id_type &dest) const {
        return _M_adjacency_list.edge_id(src, dest);
    };

    bool has_edge(const node_id_type &src, const node_id_type &dest) const { return edge_id(src, dest) != NO_EDGE_ID; }

    std::span<const internal_adjacency_list_edge<node_id_type, edge_info_type>> outgoing_edges(const node_id_type &node) const {
        return topology().outgoing_edges(node);
    };

    distance_type path_length(const path &__route) const;

    subgraph make_subgraph(const path &__route) const;

    subgraph make_subgraph(std::vector<NodeId> &&__nodes, std::vector<EdgeId> &&__edges) const;

    graph make_graph(const subgraph &__subgraph) const;
};

static_assert(RoutableGraph<graph<int, int, int, int>>);


template<typename Nid>
std::ostream &
operator<<(std::ostream &__stream, path<Nid> &__r);

