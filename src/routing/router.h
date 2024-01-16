#pragma once

#include <cassert>

#include "../graph/graph.h"
#include "dijkstra.h"
#include "dijkstra_queues.h"

#include <deque>
#include <map>
#include <ostream>

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
    graph_type const&_graph;

    search_type _forward_search;

    node_id_type _start_node;
    node_id_type _target_node;
    node_id_type _mid_node;

    node_cost_pair_type _forward_current;

private:
    void step_forward();

    /**
     * gets the minimal distance a route via the given node can have
     * @param node
     * @return
     */
    distance_type min_route_distance(node_cost_pair_type node) const;

    /**
     * TODO implement
     */
    void compute_one_to_all();

    /**
     * TODO move from compute_route()
     */
    void compute_one_to_one();

public:
    static constexpr size_t SIZE_PER_NODE = 2 * Dijkstra::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = 2 * Dijkstra::SIZE_PER_EDGE;

    explicit router(Graph const&graph);

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
    void compute_route();

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

    distance_type forward_distance() const { return _forward_current.distance(); };

    node_cost_pair_type forward_current() const { return _forward_current; };

    auto&& forward_labels() const { return _forward_search.labels(); }

    auto&& forward_search() const { return _forward_search; }

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
    typename Graph::subgraph_type shortest_path_tree(size_t max_tree_size = 50000000) const;
};
