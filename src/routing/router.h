#pragma once


template<typename Graph, typename Dijkstra>
class router {
public:
    using graph_type = Graph;
    using search_type = Dijkstra;
    using labels_type = typename search_type::labels_type;
    using node_id_type = typename graph_type::node_id_type;
    using distance_type = typename graph_type::distance_type;

    using node_cost_pair_type = typename search_type::node_cost_pair_type;
protected:
    std::shared_ptr<graph_type> _graph;

    search_type _forward_search;

    node_id_type _start_node;
    node_id_type _target_node;

    node_cost_pair_type _forward_current;

public:
    static constexpr size_t SIZE_PER_NODE = 2 * Dijkstra::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = 2 * Dijkstra::SIZE_PER_EDGE;

    explicit router(std::shared_ptr<Graph> graph);

    router(const router&other) = delete;

    router(router&&other) noexcept;

    router& operator=(const router&other) = delete;

    router& operator=(router&&other) noexcept = default;

    ~router() = default;

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
     *
     */
    void step_forward();

    /**
     *
     * @return
     */
    bool done();

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
    distance_type distance(const node_id_type& node) const;

    distance_type forward_distance() const;

    node_cost_pair_type const& forward_current() const;

    auto&& forward_labels() const;

    auto&& forward_search() const;

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
     * returns the trees of the search
     * @return
     */
    typename Graph::subgraph_type shortest_path_tree(size_t max_tree_size = std::numeric_limits<size_t>::max()) const;
};
