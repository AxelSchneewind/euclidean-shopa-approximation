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

template<typename Graph, typename Queue, typename UseEdge>
std::span<const node_id_t>
dijkstra<Graph, Queue, UseEdge>::Labels::all_visited () const
{
  return std::span<const node_id_t> (_M_visited.begin (), _M_visited.end ());
}

template<typename Graph, typename Queue, typename UseEdge>
const node_id_t &
dijkstra<Graph, Queue, UseEdge>::Labels::predecessor (const node_id_t &node) const
{
  if (node == NO_NODE_ID) return NO_NODE_ID;
  return _M_predecessor[node];
}
template<typename Graph, typename Queue, typename UseEdge>
const distance_t &
dijkstra<Graph, Queue, UseEdge>::Labels::distance (const node_id_t &node) const
{
  if (node == NO_NODE_ID) return DISTANCE_INF;
  return _M_distance[node];
}

template<typename Graph, typename Queue, typename UseEdge>
dijkstra<Graph, Queue, UseEdge>::dijkstra (const std::shared_ptr<const Graph> &__graph,
				  const adjacency_list<typename Graph::edge_info_type> &__adj_list)
  : _M_adj_list (__adj_list), _M_graph (__graph), _M_labels (this), _M_use_edge (__graph.get()),
    _M_queue (Queue (__graph))
{
  init (NO_NODE_ID);
}

template<typename Graph, typename Queue, typename UseEdge>
dijkstra<Graph, Queue, UseEdge>::dijkstra (dijkstra<Graph, Queue, UseEdge> &&other) noexcept
  : _M_adj_list (other._M_adj_list), _M_labels (std::move (other._M_labels)), _M_queue (std::move (other._M_queue))
{}

template<typename Graph, typename Queue, typename UseEdge>
const dijkstra<Graph, Queue, UseEdge>::node_cost_pair &
dijkstra<Graph, Queue, UseEdge>::current () const
{
  return _M_queue.top ();
}

template<typename Graph, typename Queue, typename UseEdge>
bool
dijkstra<Graph, Queue, UseEdge>::Labels::reached (const node_id_t &node) const
{
  assert(node != NO_NODE_ID);
  return _M_distance[node] != DISTANCE_INF && _M_predecessor[node] != NO_NODE_ID;
}

template<typename Graph, typename Queue, typename UseEdge>
bool
dijkstra<Graph, Queue, UseEdge>::reached (const node_id_t &node) const
{
  return _M_labels.reached (node);
}

template<typename Graph, typename Queue, typename UseEdge>
bool
dijkstra<Graph, Queue, UseEdge>::queue_empty () const
{
  return _M_queue.empty ();
}

template<typename Graph, typename Queue, typename UseEdge>
void
dijkstra<Graph, Queue, UseEdge>::Labels::label (const dijkstra<Graph, Queue, UseEdge>::node_cost_pair &node_cost_pair)
{
  if (_M_predecessor[node_cost_pair.node] == NO_NODE_ID)
  	_M_visited.push_back (node_cost_pair.node);
  _M_distance[node_cost_pair.node] = node_cost_pair.distance;
  _M_predecessor[node_cost_pair.node] = node_cost_pair.predecessor;
}

template<typename Graph, typename Queue, typename UseEdge>
void
dijkstra<Graph, Queue, UseEdge>::init (node_id_t start_node, node_id_t target_node)
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

template<typename Graph, typename Queue, typename UseEdge>
void
dijkstra<Graph, Queue, UseEdge>::expand (const dijkstra<Graph, Queue, UseEdge>::node_cost_pair &node)
{
  auto edges = _M_adj_list.forward ().node_edges (node.node);
  for (const adjacency_list_edge<typename Graph::edge_info_type> &edge : edges)
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
template<typename Graph, typename Queue, typename UseEdge>
void
dijkstra<Graph, Queue, UseEdge>::step ()
{
  // remove already settled nodes
  while (!_M_queue.empty () && current().distance >= _M_labels.distance (current ().node))
    _M_queue.pop ();

  if (_M_queue.empty ())
    return;

  node_cost_pair node = current ();
  expand (node);

  // label current node
  _M_labels.label (node);

  // remove current node
  _M_queue.pop ();
}

template<typename Graph, typename Queue, typename UseEdge>
dijkstra<Graph, Queue, UseEdge>::Labels::Labels (dijkstra<Graph, Queue, UseEdge> *d)
  : d (d), _M_predecessor (d->_M_graph->node_count (), NO_NODE_ID),
    _M_distance (d->_M_graph->node_count (), DISTANCE_INF)
{
  _M_visited.reserve(100000);
}

template<typename Graph, typename Queue, typename UseEdge>
void
dijkstra<Graph, Queue, UseEdge>::Labels::init (node_id_t __start_node, node_id_t __target_node)
{
  for (size_t index = 0; index < _M_visited.size (); ++index)
  {
    node_id_t node = _M_visited[index];
    _M_predecessor[node] = NO_NODE_ID;
    _M_distance[node] = DISTANCE_INF;
  }

  _M_visited.clear ();
}
