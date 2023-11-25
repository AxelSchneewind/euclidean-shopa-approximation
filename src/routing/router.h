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
private:
    Graph const& _M_graph_ptr;

    Dijkstra _M_forward_search;
    Dijkstra _M_backward_search;

    graph_type::node_id_type _M_start_node;
    graph_type::node_id_type _M_target_node;
    graph_type::node_id_type _M_mid_node;

    void step_forward();

    void step_backward();

    /**
     * gets the minimal distance a route via the given node can have
     * @param node
     * @return
     */
    Graph::distance_type min_route_distance(Dijkstra::node_cost_pair_type __node) const;

public:
    static constexpr size_t SIZE_PER_NODE = 2 * Dijkstra::SIZE_PER_NODE;
    static constexpr size_t SIZE_PER_EDGE = 2 * Dijkstra::SIZE_PER_EDGE;


    explicit router(Graph const& __graph);

    router(const router &__other) = delete;

    router(router &&__other) noexcept;

    router &operator=(const router &__other) = delete;

    router &operator=(router &&__other) noexcept = default;

    ~router() = default;

    /**
     * prepares this router to compute a route from start to target node.
     * @param start_node
     * @param target_node
     */
    void init(typename Graph::node_id_type __start_node, typename Graph::node_id_type __target_node);

    /**
     * calculates a one to one route using bidirectional dijkstra. init() has to be called first
     */
    void compute_route();

    /**
     * returns the distance of the calculated route
     * @return
     */
    typename Graph::distance_type distance() const;

    /**
     * returns the distance of the given node (or infinity if it has not been found by both searches)
     * @param node
     * @return
     */
    typename Graph::distance_type distance(const typename Graph::node_id_type &__node) const;

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
     * returns the tree of all visited nodes
     * @return
     */
    typename Graph::subgraph_type shortest_path_tree() const;

    /**
     *
     * returns the node where _M_forward_search and backward search met
     */
    typename Graph::node_id_type mid_node() const;
};
