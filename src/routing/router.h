#pragma once

#include <cassert>

#include "../graph/graph.h"
#include "dijkstra.h"
#include "dijkstra_queues.h"

#include <deque>
#include <map>
#include <ostream>

template <typename Graph, typename Dijkstra> class router
{
protected:
  std::shared_ptr<const Graph> graph;

  Dijkstra forward;
  Dijkstra backward;

  // ids
  Graph::node_id_type start_node;
  Graph::node_id_type target_node;

  /*
   * node where topology and backward search met
   */
  Graph::node_id_type _mid_node;

  void step_forward ();
  void step_backward ();

  /*
   * gets the minimal distance a route via the given node can have
   */
  distance_t min_route_distance (const Graph::node_id_type &node) const;

public:
  explicit router (std::shared_ptr<const Graph> graph);
  router (router &&other) noexcept;

  ~router () = default;

  void init(Graph::node_id_type start_node, Graph::node_id_type target_node);

  /**
   * calculates a one to one route using bidirectional dijkstra
   */
  void compute_route ();

  /**
   * returns the distance of the calculated route
   * @return
   */
  Graph::distance_type distance () const;
  /**
   * returns the distance of the given node (or infinity if it has not been found by both searches)
   * @param node
   * @return
   */
  Graph::distance_type distance (const Graph::node_id_type &node) const;

  /**
   * checks if a valid route has been found
   */
  bool route_found () const;

  /**
   *
   * returns the nodes of the path from start to target node
   * @return
   */
  Graph::path route () const;

  Graph::subgraph shortest_path_tree () const;

  // returns the node where topology and backward search met
  Graph::node_id_type mid_node () const;
};
