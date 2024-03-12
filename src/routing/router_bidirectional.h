#pragma once

template<typename Graph, typename Dijkstra>
class bidirectional_router : public router<Graph, Dijkstra> {
public:
    using base = router<Graph, Dijkstra>;
    using graph_type = Graph;
    using search_type = Dijkstra;
    using labels_type = typename router<Graph, Dijkstra>::labels_type;
    using node_id_type = typename router<Graph, Dijkstra>::node_id_type;
    using distance_type = typename router<Graph, Dijkstra>::distance_type;

    using node_cost_pair_type = typename router<Graph, Dijkstra>::node_cost_pair_type;

private:
    Dijkstra _backward_search;

    node_cost_pair_type _backward_current;

    void step_forward();

    void step_backward();

    /**
     * gets the minimal distance a route via the given node can have
     * @param node
     * @return
     */
    distance_type min_route_distance(node_cost_pair_type node) const;

public:
    static constexpr size_t SIZE_PER_NODE = 2 * Dijkstra::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = 2 * Dijkstra::SIZE_PER_EDGE;


    explicit bidirectional_router(std::shared_ptr<Graph> graph);

    bidirectional_router(const bidirectional_router&other) = delete;

    bidirectional_router(bidirectional_router&&other) noexcept;

    bidirectional_router& operator=(const bidirectional_router&other) = delete;

    bidirectional_router& operator=(bidirectional_router&&other) noexcept = default;

    ~bidirectional_router() = default;

    /**
     * prepares this router to compute a route from start to target node.
     * @param start_node
     * @param target_node
     */
    void init(node_id_type start_node, node_id_type target_node);

    /**
     * calculates a one to one route using bidirectional dijkstra. init() has to be called first
     */
    void compute();

    /**
     * returns the distance of the calculated route
     * @return
     */
    distance_type distance() const;

    /**
     * returns the distance of the given node (or infinity if it has not been found by both searches)
     * @param node
     * @return
     */
    distance_type distance(const node_id_type&node) const;

    distance_type forward_distance() const { return base::_forward_current.distance(); };

    distance_type backward_distance() const { return _backward_current.distance(); };

    node_cost_pair_type forward_current() const { return base::_forward_current; };

    node_cost_pair_type backward_current() const { return _backward_current; };

    auto&& forward_labels() const { return base::_forward_search.labels(); }

    auto&& backward_labels() const { return _backward_search.labels(); }

    auto&& forward_search() const { return base::_forward_search; }

    auto&& backward_search() const { return _backward_search; }

    /**
     * checks if a valid route has been found
     */
    bool route_found() const;

    /**
     *
     * returns the nodes of the path from start to target node
     * @return
     */
    typename Graph::path_type route() const;

    /**
     * returns the trees of the forward and backward search
     * @return
     */
    typename Graph::subgraph_type shortest_path_tree(size_t max_tree_size = 10000000) const;

    /**
     * returns the tree of the forward search
     * @return
     */
    typename Graph::subgraph_type tree_forward(size_t max_tree_size = 5000000) const {
        return base::_forward_search.shortest_path_tree(max_tree_size);
    }

    typename Graph::subgraph_type tree_backward(size_t max_tree_size = 5000000) const {
        return _backward_search.shortest_path_tree(max_tree_size);
    };

    /**
     *
     * returns the node where forward search and backward search met
     */
    node_id_type mid_node() const;
};
