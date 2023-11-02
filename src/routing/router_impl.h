#pragma once

#include <cassert>

#include "router.h"

#include "../graph/graph_impl.h"
#include "dijkstra_impl.h"
#include <deque>
#include <map>
#include <ostream>

#include "dijkstra_queues.h"

template <typename Graph, typename Dijkstra>
distance_t
router<Graph, Dijkstra>::min_route_distance (const node_id_t &node) const
{
  distance_t result = 0;

  // TODO check with CH

  if (Dijkstra::search_symmetric)
  {
    // if node is labelled in topology search, it already has its minimal distance and routes using it must be longer
    if (forward.reached (node))
      result += forward.labels ().distance (node);
    else if (!forward.queue_empty ())
      // otherwise, its distance must be at least the one of the current node
      result += forward.current ().distance;

    if (backward.reached (node))
      result += backward.labels ().distance (node);
    else if (!backward.queue_empty ())
      result += backward.current ().distance;
  }
  else
  {
    // if labels are not necessarily optimal, only use distances if the node has been labelled already
    if (forward.reached (node))
      result += forward.labels ().distance (node);
    if (backward.reached (node))
      result += backward.labels ().distance (node);

    if (!forward.reached (node) && !backward.reached (node))
      result += std::max (forward.current ().distance, backward.current ().distance);
  }

  return result;
}

template <typename Graph, typename Dijkstra>
Graph::subgraph
router<Graph, Dijkstra>::shortest_path_tree () const
{
  std::vector<node_id_t> nodes;
  std::vector<edge_id_t> edges;

  // add nodes and edges of topology dijkstra
  for (auto n : forward.labels ().all_visited ())
  {
    nodes.push_back (n);
    node_id_t pred = forward.labels ().predecessor (n);
    edge_id_t edge = graph->topology().edge_id(pred, n);
    edges.push_back (edge);
  }

  // add nodes and edges of backward dijkstra
  for (auto n : backward.labels ().all_visited ())
  {
    nodes.push_back (n);

    node_id_t succ = backward.labels ().predecessor (n);
    edge_id_t edge = graph->topology().edge_id(n, succ);
    edges.push_back (edge);
  }

  remove_duplicates (nodes);
  remove_duplicates (edges);

  return graph->make_subgraph (std::move (nodes), std::move (edges));
}

template <typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router (std::shared_ptr<const Graph> graph)
  : graph (graph), forward (this->graph, graph->list()), backward (this->graph, graph->list().inverse()),
    start_node (NO_NODE_ID), target_node (NO_NODE_ID), _mid_node (NO_NODE_ID)
{}

template <typename Graph, typename Dijkstra>
router<Graph, Dijkstra>::router (router &&routing) noexcept
  : graph (routing.graph), forward (std::move (routing.forward)), backward (std::move (routing.backward)),
    start_node (routing.start_node), target_node (routing.target_node), _mid_node (routing._mid_node)
{}

template <typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::step_forward ()
{
  assert (!forward.queue_empty ());

  node_cost_pair current = forward.current ();

  forward.step ();

  // check if searches met and provide best result so far
  if (backward.reached (current.node) && distance (current.node) < distance ())
    _mid_node = current.node;
}

template <typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::step_backward ()
{
  assert (!backward.queue_empty ());

  node_cost_pair current = backward.current ();

  backward.step ();

  // check if searches met and provide best result yet
  if (forward.reached (current.node) && distance (current.node) < distance ())
    _mid_node = current.node;
}

template <typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::compute_route ()
{
  // check args
  if (start_node >= graph->node_count () || target_node >= graph->node_count () || start_node < 0
      || target_node < 0 || start_node == NO_NODE_ID || target_node == NO_NODE_ID)
    throw;

  // TODO: 2 threads performing topology and backward search simultaneously?
  bool done = false;
  while (!done)
  {
    bool const fwd_done = forward.queue_empty () || min_route_distance (forward.current().node) > distance();
    bool const bwd_done = backward.queue_empty () || min_route_distance (backward.current().node) > distance();

    // check if no better route can be found
    if (fwd_done && bwd_done)
      done = true;

    if (!fwd_done)
      step_forward ();
    if (!bwd_done)
      step_backward ();
  }

  if (!forward.queue_empty ())
    step_forward ();
  if (!backward.queue_empty ())
    step_backward ();
}

template <typename Graph, typename Dijkstra>
distance_t
router<Graph, Dijkstra>::distance (const node_id_t &node) const
{
  if (node == NO_NODE_ID || !forward.labels ().reached (node) || !backward.labels ().reached (node))
    return DISTANCE_INF;
  return forward.labels ().distance (node) + backward.labels ().distance (node);
}

template <typename Graph, typename Dijkstra>
distance_t
router<Graph, Dijkstra>::distance () const
{
  if (_mid_node == NO_NODE_ID)
    return DISTANCE_INF;
  return forward.labels ().distance (_mid_node) + backward.labels ().distance (_mid_node);
}

template <typename Graph, typename Dijkstra>
Graph::path
router<Graph, Dijkstra>::route () const
{
  if (_mid_node == NO_NODE_ID)
    throw;

  node_id_t fwd_node = _mid_node;
  node_id_t bwd_node = _mid_node;

  std::deque<node_id_t> p;
  p.push_front (_mid_node);

  while (fwd_node != NO_NODE_ID && fwd_node != start_node && fwd_node != forward.labels ().predecessor (fwd_node))
  {
    assert (graph->topology().has_edge (forward.labels ().predecessor (fwd_node), fwd_node));

    fwd_node = forward.labels ().predecessor (fwd_node);
    assert(fwd_node != NO_NODE_ID);

    p.push_front (fwd_node);
  }

  while (bwd_node != NO_NODE_ID && bwd_node != target_node && bwd_node != backward.labels ().predecessor (bwd_node))
  {
    bwd_node = backward.labels ().predecessor (bwd_node);
    assert(bwd_node != NO_NODE_ID);

    p.push_back (bwd_node);
  }

  return { std::vector<node_id_t> (p.begin (), p.end ()) };
};

template <typename Graph, typename Dijkstra>
node_id_t
router<Graph, Dijkstra>::mid_node () const
{
  return _mid_node;
};

template <typename Graph, typename Dijkstra>
bool
router<Graph, Dijkstra>::route_found () const
{
  return _mid_node != NO_NODE_ID && forward.reached (_mid_node) && backward.reached (_mid_node);
}
template <typename Graph, typename Dijkstra>
void
router<Graph, Dijkstra>::init (node_id_t __start_node, node_id_t __target_node)
{
  // check args
  if (__start_node >= graph->node_count () || __target_node >= graph->node_count () || __start_node < 0
      || __target_node < 0 || __start_node == NO_NODE_ID || __target_node == NO_NODE_ID)
    throw;

  start_node = __start_node;
  target_node = __target_node;
  forward.init (start_node, target_node);
  backward.init (target_node, start_node);

  _mid_node = NO_NODE_ID;
}
