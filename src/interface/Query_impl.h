#pragma once

#include <chrono>
#include <iostream>

#include "../graph/base_types.h"

#include "../graph/graph.h"
#include "Graph.h"

template<typename GraphT, typename RouterT>
Result::Result(GraphT const &graph, QueryImplementation<GraphT> query, RouterT const &router,
               std::chrono::duration<double, std::milli> duration)
        : Base<ResultInterface>(ResultImplementation<GraphT>(graph, std::move(query), router, duration)) {};


template<RoutableGraph GraphT>
template<typename RouterT>
ResultImplementation<GraphT>::ResultImplementation(const GraphT &graph, QueryImplementation<GraphT> query,
                                                   const RouterT &router,
                                                   std::chrono::duration<double, std::milli> duration)
        : _query(query),
          _route_found(router.route_found()),
          _path(std_graph_t::make_graph(graph, graph.make_subgraph(router.route()))),
          _tree_forward(std_graph_t::make_graph(graph, router.shortest_path_tree(query.max_tree_size()))),
          _tree_backward(std_graph_t::make_graph(graph, router.shortest_path_tree(0))),
          _nodes_visited(_tree_forward.node_count()),
          _distance(router.distance()),
          _duration(duration),
          _pull_count(router.forward_search().pull_count()),
          _push_count(router.forward_search().push_count()),
          _edges_visited(router.forward_search().edges_checked()),
          _queue_max_size(router.forward_search().queue().max_size()),
          _base_node_count{0},
          _boundary_node_count{0},
          _steiner_point_count{0},
          _base_node_neighbor_count{0},
          _boundary_node_neighbor_count{0},
          _steiner_point_neighbor_count{0} {
  if constexpr (requires (typename RouterT::search_type::neighbor_getter_type const& n) { n.base_node_count(); }) {
      _base_node_count = router.forward_search().neighbors().base_node_count();
      _boundary_node_count = router.forward_search().neighbors().boundary_node_count();
      _steiner_point_count = router.forward_search().neighbors().steiner_point_count();
      _base_node_neighbor_count = router.forward_search().neighbors().base_node_neighbor_count();
      _boundary_node_neighbor_count = router.forward_search().neighbors().boundary_node_neighbor_count();
      _steiner_point_neighbor_count = router.forward_search().neighbors().steiner_point_neighbor_count();
  }
};


template<RoutableGraph GraphT>
std_graph_t make_beeline(GraphT const &graph, typename GraphT::node_id_type from, typename GraphT::node_id_type to) {
    std::vector<node_t> nodes{graph.node(from), graph.node(to)};
    unidirectional_adjacency_list<int, edge_t>::adjacency_list_builder edges(2);
    edges.add_edge(0, 1, distance(nodes[0].coordinates, nodes[1].coordinates));
    edges.add_edge(1, 0, distance(nodes[0].coordinates, nodes[1].coordinates));
    std_graph_t beeline = std_graph_t::make_graph(std::move(nodes),
                                                  adjacency_list<int, edge_t>::make_bidirectional(edges.get()));
    return beeline;
}

template<RoutableGraph GraphT>
QueryImplementation<GraphT>::QueryImplementation(GraphT const &graph, long from, long to, RoutingConfiguration const& config)
        : _from(from), _to(to), _from_internal(from), _to_internal(to),
          _beeline_distance(distance(graph.node(_from_internal).coordinates, graph.node(_to_internal).coordinates)),
          _beeline(make_beeline(graph, _from_internal, _to_internal)),
          _configuration(config) {};

template<>
QueryImplementation<steiner_graph>::QueryImplementation(steiner_graph const &graph, long from, long to, RoutingConfiguration const& config)
        : _from(from), _to(to), _from_internal(graph.from_base_node_id(from)),
          _to_internal(graph.from_base_node_id(to)),
          _beeline_distance(distance(graph.node(_from_internal).coordinates, graph.node(_to_internal).coordinates)),
          _beeline(make_beeline(graph, _from_internal, _to_internal)),
          _configuration(config) {};

template<RoutableGraph GraphT>
QueryImplementation<GraphT>::QueryImplementation() : _from_internal{none_value<node_id_type>},
                                                     _to_internal{none_value<node_id_type>}, _from{none_value<long>},
                                                     _to{none_value<long>} {}



template<RoutableGraph GraphT>
void ResultImplementation<GraphT>::write(table &out) const {
    out.put(Statistics::COST, distance());
    out.put(Statistics::TREE_SIZE, nodes_visited());
    // out.put(Statistics::PATH, path());
    out.put(Statistics::TIME, duration());
    out.put(Statistics::QUEUE_PULL_COUNT, pull_count());
    out.put(Statistics::QUEUE_PUSH_COUNT, push_count());
    out.put(Statistics::QUEUE_MAX_SIZE, queue_max_size());
    out.put(Statistics::EDGES_CHECKED, edges_visited());

    out.put(Statistics::NEIGHBORS_BASE_NODE_COUNT, _base_node_count);
    out.put(Statistics::NEIGHBORS_BASE_NODE_NEIGHBORS_COUNT, _base_node_neighbor_count);
    out.put(Statistics::NEIGHBORS_BOUNDARY_NODE_COUNT, _boundary_node_count);
    out.put(Statistics::NEIGHBORS_BOUNDARY_NODE_NEIGHBORS_COUNT, _boundary_node_neighbor_count);
    out.put(Statistics::NEIGHBORS_STEINER_POINT_COUNT, _steiner_point_count);
    out.put(Statistics::NEIGHBORS_STEINER_POINT_NEIGHBORS_COUNT, _steiner_point_neighbor_count);
}
