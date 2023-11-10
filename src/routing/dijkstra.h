#pragma once

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../graph/unidirectional_adjacency_list.h"

#include "dijkstra_concepts.h"

#include <queue>
#include <vector>

#include <cmath>
#include <concepts>
#include <iostream>


template <RoutableGraph G, DijkstraQueue<G> Q, typename UseEdge, DijkstraLabels L> class dijkstra
{
    static_assert(Topology<typename G::topology_type>);
public:
  using type = dijkstra<G, Q, UseEdge, L>;
  using node_cost_pair = Q::value_type;
  using edge_info_type = G::edge_info_type;
  using node_id_type =  G::node_id_type;

  // determines optimality of labels depending on whether the graph allows shortcuts
  // TODO find more elegant way for this
  static constexpr bool search_symmetric = (typeid (edge_info_type) != typeid (ch_edge_t));

private:
  std::shared_ptr<const G> _M_graph;
  G::topology_type _M_topology;

  node_id_type _M_start_node;
  node_id_type _M_target_node;

  Q _M_queue;
  UseEdge _M_use_edge;

  L _M_labels;

  // add reachable (and not settled) nodes to active nodes in queue
  void expand (const node_id_type &__node);

public:

  dijkstra (dijkstra &&__other) noexcept;
  dijkstra (const dijkstra &__other) = default;

  // constructs a dijkstra object for the given graph and m_adj_list
  explicit dijkstra (const std::shared_ptr<const G> &__graph, const G::topology_type &__adj_list);
  ~dijkstra () = default;

  dijkstra<G, Q, UseEdge, L> &operator= (dijkstra<G, Q, UseEdge, L> &&other) = default;
  dijkstra<G, Q, UseEdge, L> &operator= (const dijkstra<G, Q, UseEdge, L> &other) = default;


  typename G::node_id_type source () const { return _M_start_node; }
  typename G::node_id_type target () const { return _M_target_node; }

  /**
   * gets the stored labels
   * @return
   */
  const L &labels () const { return _M_labels; }

  /**
   * init one to one
   *
   * @param start_node
   * @param target_node
   */
  void init (node_id_type __start_node, node_id_type __target_node = none_value<node_id_type>());

  /**
   * get the current node without removing from queue
   * @return
   */
  node_cost_pair current () const;

  /**
   * step to current node in queue, i.e. store label and add adjacent nodes
   */
  void step ();

  /**
   * function to check whether search is finished (queue empty)
   * @return
   */
  bool queue_empty () const;

  /**
   * check if node is reached (labelled)
   * @param node
   * @return
   */
  bool reached (G::node_id_type __node) const;
};
