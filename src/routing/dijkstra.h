#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../graph/unidirectional_adjacency_list.h"

#include "dijkstra_concepts.h"

#include <queue>
#include <vector>

#include <cmath>
#include <concepts>
#include <iostream>


template<RoutableGraph G, DijkstraQueue<G> Q, typename UseEdge, DijkstraLabels L>
class dijkstra {
    static_assert(Routable<typename G::topology_type>);
public:
    using type = dijkstra<G, Q, UseEdge, L>;
    using node_cost_pair = typename Q::value_type;
    using node_id_type = typename G::node_id_type;

    using graph_type = G;
    using use_edge_type = UseEdge;
    using queue_type = Q;
    using labels_type = L;

    // determines optimality of labels depending on whether the graph allows shortcuts
    // TODO find more elegant way for this
    static constexpr bool search_symmetric = (typeid(typename G::edge_info_type) != typeid(ch_edge_t));

    static constexpr size_t SIZE_PER_NODE = L::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = L::SIZE_PER_EDGE;

private:
    std::shared_ptr<const G> _M_graph;

    node_id_type _M_start_node;
    node_id_type _M_target_node;

    Q _M_queue;
    UseEdge _M_use_edge;

    L _M_labels;

    // add reachable (and not settled) nodes to active nodes in queue
    void expand(node_cost_pair __node);

public:

    dijkstra(dijkstra &&__other) noexcept;

    dijkstra(const dijkstra &__other) = delete;

    // constructs a dijkstra object for the given graph and m_adj_list
    explicit dijkstra(std::shared_ptr<const G> __graph);

    // constructs a dijkstra object for the given graph and m_adj_list
    explicit dijkstra(std::shared_ptr<const G> __graph, Q &&__queue, UseEdge &&__use_edge, L &&__labels)
            : _M_graph(__graph), _M_queue(std::move(__queue)), _M_use_edge(std::move(__use_edge)),
              _M_labels(std::move(__labels)) {};

    ~dijkstra() = default;

    dijkstra<G, Q, UseEdge, L> &operator=(dijkstra<G, Q, UseEdge, L> &&__other) noexcept = default;

    dijkstra<G, Q, UseEdge, L> &operator=(const dijkstra<G, Q, UseEdge, L> &__other) = delete;


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
    void init(node_id_type __start_node, node_id_type __target_node = none_value<node_id_type>());

    /**
     * get the current node without removing from queue
     * @return
     */
    node_cost_pair current() const;

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
