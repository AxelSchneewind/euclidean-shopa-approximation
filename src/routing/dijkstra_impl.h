#pragma once

#include "dijkstra.h"

#include "../graph/adjacency_list.h"
#include "../graph/base_types.h"
#include "../graph/graph.h"
#include "../graph/unidirectional_adjacency_list.h"

#include <queue>
#include <vector>

#include <cmath>
#include <concepts>
#include <iostream>


template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
dijkstra<G, Queue, U, L>::dijkstra (const std::shared_ptr<const G> &__graph,
				  const adjacency_list<typename G::edge_info_type> &__adj_list)
  : _M_adj_list (__adj_list), _M_graph (__graph), _M_labels (__graph.get()), _M_use_edge (__graph.get()),
    _M_queue (Queue (__graph))
{
  init (NO_NODE_ID);
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
dijkstra<G, Queue, U, L>::dijkstra (dijkstra<G, Queue, U, L> &&other) noexcept
  : _M_adj_list (other._M_adj_list), _M_labels (std::move (other._M_labels)), _M_graph(std::move(other._M_graph)), _M_queue (std::move (other._M_queue))
{}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
const dijkstra<G, Queue, U, L>::node_cost_pair &
dijkstra<G, Queue, U, L>::current () const
{
  return _M_queue.top ();
}


template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
bool
dijkstra<G, Queue, U, L>::reached (const node_id_t &node) const
{
  return _M_labels.reached (node);
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
bool
dijkstra<G, Queue, U, L>::queue_empty () const
{
  return _M_queue.empty ();
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
void
dijkstra<G, Queue, U, L>::init (node_id_t start_node, node_id_t target_node)
{
  _M_target_node = target_node;

  _M_start_node = start_node;

  _M_queue.init(start_node, target_node);
  _M_labels.init (start_node, target_node);

  // add start node to queue
  if (start_node != NO_NODE_ID)
  {
    _M_queue.push (start_node, start_node, 0);
  }
}

template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
void
dijkstra<G, Queue, U, L>::expand (const dijkstra<G, Queue, U, L>::node_cost_pair &node)
{
  auto edges = _M_adj_list.forward ().node_edges (node.node);
  for (const adjacency_list_edge<typename G::edge_info_type> &edge : edges)
  {
    // ignore certain edges
    if (!_M_use_edge (node.node, edge))
      continue;

    const node_id_t &successor = edge.destination;
    const distance_t &successor_cost = _M_labels.distance (successor);
    const distance_t new_cost = node.distance + edge.info.cost;

    if (new_cost < successor_cost)
    {
      // (re-)insert node into the queue with updated priority
      _M_queue.push (successor, node.node, new_cost);
    }
  }
}
template<RoutableGraph G, DijkstraQueue<G> Queue, typename U, DijkstraLabels L>
void
dijkstra<G, Queue, U, L>::step ()
{
  // remove already settled nodes
  while (!_M_queue.empty () && current().distance >= _M_labels.distance (current ().node))
    _M_queue.pop ();

  if (_M_queue.empty ())
    return;

  node_cost_pair ncp = current ();

  // label current node
  _M_labels.label (ncp);

  // expand to adjacent nodes
  expand (ncp);

  // remove current node
  _M_queue.pop ();
}
