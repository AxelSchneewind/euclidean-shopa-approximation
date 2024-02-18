#pragma once

#include "dijkstra_concepts.h"

template<RoutableGraph G, DijkstraQueue<G> Q,
        DijkstraLabels<typename G::node_id_type, typename Q::value_type, typename Q::value_type> L,
        NeighborsGetter<typename Q::value_type> N>
class dijkstra {
public:
    using type = dijkstra<G, Q, L, N>;
    using node_cost_pair_type = typename Q::value_type;
    using node_id_type = typename G::node_id_type;
    using distance_type = typename G::distance_type;

    using graph_type = G;
    using queue_type = Q;
    using labels_type = L;
    using neighbor_getter_type = N;

    static constexpr size_t SIZE_PER_NODE = L::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = L::SIZE_PER_EDGE;

private:
    G const &_graph;

    node_id_type _start_node;
    node_id_type _target_node;

    distance_type _min_distance;

    Q _queue;
    L _labels;

    N _neighbors;

    std::size_t _push_count{0};
    std::size_t _pull_count{0};
    std::size_t _edges_checked{0};

    // add reachable (and not settled) nodes to active nodes in queue
    void expand(node_cost_pair_type node);

public:

    dijkstra(dijkstra &&other) noexcept;

    dijkstra(const dijkstra &other) = delete;


    // constructs a dijkstra object for the given graph
    explicit dijkstra(G const &graph)
            : _graph(graph), _queue{_graph},
              _labels(_graph), _neighbors{_graph, _labels} {};

   //  TODO
    explicit dijkstra(G const &graph, dijkstra&& other)
            : _graph(graph), _queue{std::move(other._queue)},
              _labels(_graph, std::move(other._labels)), _neighbors(_graph, _labels) {};

    explicit dijkstra(G const &graph, Q &&queue, L &&labels, N &&neighbors)
            : _graph(graph), _queue(std::move(queue)),
              _labels(std::move(labels)), _neighbors(std::move(neighbors)) {};

    ~dijkstra() = default;

    dijkstra &operator=(dijkstra &&other) noexcept = default;

    dijkstra &operator=(dijkstra const &other) = delete;

    distance_type min_path_length() const;

    node_id_type source() const { return _start_node; }

    node_id_type target() const { return _target_node; }

    const Q &queue() const { return _queue; }

    Q &queue() { return _queue; }

    const L &labels() const { return _labels; }
    const N &neighbors() const { return _neighbors; }

    L &labels() { return _labels; }

    typename L::label_type const& get_label(node_id_type node) const { return _labels.at(node); }

    /**
     * init one to one
     *
     * @param start_node
     * @param target_node
     */
    void init(node_id_type start_node, node_id_type target_node = none_value<node_id_type>);

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
    G::subgraph_type shortest_path_tree(int max_node_count = 1000000) const;

    /**
     * returns the path from source to target node (if found, otherwise throws exception)
     * @param target
     * @return
     */
    G::path_type path(node_id_type target) const;

    std::size_t push_count() const {return _push_count;};
    std::size_t pull_count() const {return _pull_count;};
    std::size_t edges_checked() const {return _edges_checked;};
};
