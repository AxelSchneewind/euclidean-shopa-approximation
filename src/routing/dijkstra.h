#pragma once

#include "dijkstra_concepts.h"

template<typename T, typename G>
concept EdgePredicate = std::predicate<T, typename G::node_id_type, internal_adjacency_list_edge<typename G::node_id_type, typename G::edge_info_type>>;


template<typename L, typename NodeId, typename NodeCostPair, typename Label>
concept PreliminaryLabels = requires(L l, NodeId id, NodeCostPair ncp) {
    l.label_preliminary(id, ncp);
    { l.get_preliminary(id) } -> std::convertible_to<Label>;
};


template<RoutableGraph G, DijkstraQueue<G> Q,
        EdgePredicate<G> UseEdge,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L>
class dijkstra {
    static_assert(Routable<typename G::topology_type>);
public:
    using type = dijkstra<G, Q, UseEdge, L>;
    using node_cost_pair_type = typename Q::value_type;
    using node_id_type = typename G::node_id_type;

    using graph_type = G;
    using use_edge_type = UseEdge;
    using queue_type = Q;
    using labels_type = L;

    static constexpr bool preliminary_labels = PreliminaryLabels<L, node_id_type, node_cost_pair_type, typename labels_type::label_type>;

    // determines optimality of labels depending on whether the graph allows shortcuts
    // TODO find more elegant way for this
    static constexpr bool search_symmetric = (typeid(typename G::edge_info_type) != typeid(ch_edge_t));

    static constexpr size_t SIZE_PER_NODE = L::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = L::SIZE_PER_EDGE;

private:
    G const &_M_graph;

    node_id_type _M_start_node;
    node_id_type _M_target_node;

    Q _M_queue;
    UseEdge _M_use_edge;

    L _M_labels;

    // add reachable (and not settled) nodes to active nodes in queue
    void expand(node_cost_pair_type __node);

public:

    dijkstra(dijkstra &&__other) noexcept;

    dijkstra(const dijkstra &__other) = delete;


    // constructs a dijkstra object for the given graph
    explicit dijkstra(G const &__graph)
            : _M_graph(__graph), _M_queue{__graph}, _M_use_edge{__graph},
              _M_labels{__graph} {};

    explicit dijkstra(G const &__graph, Q &&__queue, UseEdge &&__use_edge, L &&__labels)
            : _M_graph(__graph), _M_queue(std::move(__queue)), _M_use_edge(std::move(__use_edge)),
              _M_labels(std::move(__labels)) {};

    ~dijkstra() = default;

    dijkstra<G, Q, UseEdge, L> &operator=(dijkstra<G, Q, UseEdge, L> &&__other) noexcept = default;

    dijkstra<G, Q, UseEdge, L> &operator=(const dijkstra<G, Q, UseEdge, L> &__other) = delete;


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


// template<typename T, typename Distance>
// concept Frontier = requires(T t, Distance d) { t.set_frontier_distance(d); };
//
// template<typename T, typename NodeId, typename Ncp, typename Label, typename Distance>
// concept FrontierLabels =
// DijkstraLabels<T, NodeId, Ncp, Label> && requires(T t, Distance d) { t.set_frontier_distance(d); };
//
// template<RoutableGraph G, DijkstraQueue<G> Q,
//         EdgePredicate<G> UseEdge,
//         DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L> requires FrontierLabels<L, typename G::node_id_type, typename Q::value_type, typename Q::value_type, typename G::distance_type>
// class dijkstra<G, Q, UseEdge, L>;
