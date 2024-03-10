#pragma once

#include "dijkstra_concepts.h"

template< RoutableGraph G
        , DijkstraQueue<G> Q
        , DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L
        , NeighborsGetter<typename Q::value_type> N
        , typename H = no_heuristic>
class dijkstra {
public:
    using type = dijkstra<G, Q, L, N, H>;
    using node_cost_pair_type = typename Q::value_type;
    using node_id_type = typename G::node_id_type;
    using distance_type = typename G::distance_type;

    using graph_type = G;
    using queue_type = Q;
    using labels_type = L;
    using neighbor_getter_type = N;
    using heuristic_type = H;

    static constexpr size_t SIZE_PER_NODE = L::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = L::SIZE_PER_EDGE;

private:
    std::shared_ptr<G> _graph;
    std::shared_ptr<L> _labels;

    node_id_type _start_node;
    node_id_type _target_node;

    Q _queue;

    N _neighbors;
    H _heuristic;

    std::size_t _push_count{0};
    std::size_t _pull_count{0};
    std::size_t _edges_checked{0};

    // add reachable (and not settled) nodes to active nodes in queue
    void expand(node_cost_pair_type node);

public:
    dijkstra(dijkstra &&other) noexcept : _graph{std::move(other._graph)}, _labels{std::move(other._labels)}, _queue{other._queue}, _neighbors{other._neighbors}, _heuristic{other._heuristic} {};
    dijkstra(const dijkstra &other) = delete;

    // constructs a dijkstra object for the given graph
    explicit dijkstra(std::shared_ptr<G> graph);

    [[deprecated]]
    explicit dijkstra(std::shared_ptr<G> graph, Q &&queue, L &&labels, N &&neighbors);

    ~dijkstra() = default;

    dijkstra &operator=(dijkstra &&other) noexcept = default;

    dijkstra &operator=(dijkstra const &other) = delete;

    node_id_type source() const;

    node_id_type target() const;

    const Q &queue() const;
    Q &queue();

    const L &labels() const;
    L &labels();

    const N &neighbors() const;

    typename L::value_type get_label(node_id_type node) const;

    /**
     * init one to one
     *
     * @param start_node
     * @param target_node
     */
    void init(node_id_type start_node, node_id_type target_node = optional::none_value<node_id_type>);

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
    bool reached(node_id_type node) const;


    /**
     * returns the shortest path tree containing all nodes that have been settled with their minimal distances
     * @param Graph
     * @param Dijkstra
     * @return
     */
    G::subgraph_type shortest_path_tree(std::size_t max_node_count = 1000000) const;

    /**
     * returns the path from source to target node (if found, otherwise throws exception)
     * @param target
     * @return
     */
    G::path_type path(node_id_type target) const;

    std::size_t push_count() const;
    std::size_t pull_count() const;
    std::size_t edges_checked() const;
};
