#pragma once

#include <cassert>

#include "../graph/graph.h"
#include "dijkstra.h"
#include "dijkstra_queues.h"

#include <deque>
#include <map>
#include <ostream>

template <typename Graph, typename Dijkstra> class routing
{
protected:
  std::shared_ptr<const Graph> graph;

  Dijkstra forward;
  Dijkstra backward;

  // ids
  node_id_t start_node;
  node_id_t target_node;

  /*
   * node where forward and backward search met
   */
  node_id_t _mid_node;

  void step_forward ();
  void step_backward ();

  /*
   * gets the minimal distance a route via the given node can have
   */
  distance_t min_route_distance (const node_id_t &node) const;

public:
  explicit routing (std::shared_ptr<const Graph> graph);
  routing (routing &&other) noexcept;

  ~routing () = default;

  /**
   * calculates a one to one route using bidirectional dijkstra
   * @param start_node
   * @param target_node
   */
  void compute_route (node_id_t start_node, node_id_t target_node);

  /**
   * returns the distance of the calculated route
   * @return
   */
  distance_t distance () const;
  /**
   * returns the distance of the given node (or infinity if it has not been found by both searches)
   * @param node
   * @return
   */
  distance_t distance (const node_id_t &node) const;

  /**
   * checks if a valid route has been found
   */
  bool route_found () const;

  /**
   *
   * returns the nodes of the path from start to target node
   * @return
   */
  path route () const;

  subgraph shortest_path_tree () const;

  // returns the node where forward and backward search met
  node_id_t mid_node () const;
};
