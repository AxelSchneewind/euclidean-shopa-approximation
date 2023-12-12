#pragma once

#include "dijkstra_concepts.h"

template<typename T, typename G>
concept EdgePredicate = std::predicate<T, typename G::node_id_type, internal_adjacency_list_edge<typename G::node_id_type, typename G::edge_info_type>>;

template<typename T, typename NodeCostPair>
concept NeighborsGetter = requires(T t, NodeCostPair n, std::vector<NodeCostPair> &r) { t(n, r); };

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N, EdgePredicate<G> UseEdge>
class dijkstra {
    static_assert(Routable<typename G::topology_type>);
public:
    using type = dijkstra<G, Q, L, N, UseEdge>;
    using node_cost_pair_type = typename Q::value_type;
    using node_id_type = typename G::node_id_type;
    using distance_type = typename G::distance_type;

    using graph_type = G;
    using use_edge_type = UseEdge;
    using queue_type = Q;
    using labels_type = L;
    using neighbor_getter_type = N;

    static constexpr size_t SIZE_PER_NODE = L::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = L::SIZE_PER_EDGE;

private:
    G const &_M_graph;

    node_id_type _M_start_node;
    node_id_type _M_target_node;

    distance_type _M_min_distance;

    Q _M_queue;
    L _M_labels;

    N _M_neighbors;
    UseEdge _M_use_edge;

    // add reachable (and not settled) nodes to active nodes in queue
    void expand(node_cost_pair_type __node);

public:

    dijkstra(dijkstra &&__other) noexcept;

    dijkstra(const dijkstra &__other) = delete;


    // constructs a dijkstra object for the given graph
    explicit dijkstra(G const &__graph)
            : _M_graph(__graph), _M_queue{__graph},
              _M_labels{__graph}, _M_neighbors{__graph, _M_labels}, _M_use_edge{__graph} {};

    explicit dijkstra(G const &__graph, Q &&__queue, L &&__labels, N &&__neighbors, UseEdge &&__use_edge)
            : _M_graph(__graph), _M_queue(std::move(__queue)),
              _M_labels(std::move(__labels)), _M_neighbors(std::move(__neighbors)),
              _M_use_edge(std::move(__use_edge)) {};

    ~dijkstra() = default;

    dijkstra &operator=(dijkstra &&__other) noexcept = default;

    dijkstra &operator=(dijkstra const &__other) = delete;

    typename G::distance_type min_path_length() const;

    typename G::node_id_type source() const { return _M_start_node; }

    typename G::node_id_type target() const { return _M_target_node; }

    const Q &queue() const { return _M_queue; }

    Q &queue() { return _M_queue; }

    const L &labels() const { return _M_labels; }

    L &labels() { return _M_labels; }

    L::label_type get_label(G::node_id_type __node) const { return _M_labels.get(__node); }

    /**
     * init one to one
     *
     * @param start_node
     * @param target_node
     */
    void init(node_id_type __start_node, node_id_type __target_node = none_value<node_id_type>);

    /**
     * get the current node without removing from queue
     * @return
     */
    node_cost_pair_type current() const;

    /**
     * step to current node in queue, i.e. store label_type and add adjacent nodes
     */
    void step();

    /**
     * function to check whether search is finished (queue empty)
     * @return
     */
    bool queue_empty() const;

    /**
     * check if node is reached (labelled)
     * @param node
     * @return
     */
    bool reached(typename G::node_id_type __node) const;
};
