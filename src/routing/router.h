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
private:
    std::shared_ptr<const Graph> _M_graph_ptr;

    Dijkstra _M_forward_search;
    Dijkstra _M_backward_search;

    typename Graph::node_id_type _M_start_node;
    typename Graph::node_id_type _M_target_node;
    typename Graph::node_id_type _M_mid_node;

    void step_forward();

    void step_backward();

    /**
     * gets the minimal distance a route via the given node can have
     * @param node
     * @return
     */
    distance_t min_route_distance(const typename Graph::node_id_type &__node) const;

public:
    explicit router(std::shared_ptr<const Graph> __graph);

    router(const router &__other);

    router(router &&__other) noexcept;

    router &operator=(const router &__other) = default;

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
    typename Graph::path route() const;

    /**
     * returns the tree of all visited nodes
     * @return
     */
    typename Graph::subgraph shortest_path_tree() const;

    /**
     *
     * returns the node where _M_forward_search and backward search met
     */
    typename Graph::node_id_type mid_node() const;
};
