#pragma once

#include "dijkstra_concepts.h"

#include <vector>

template <RoutableGraph G, typename NodeCostPair>
class Labels
{
private:
  using node_id_type = G::node_id_type;
  using distance_type = G::distance_type;
  using node_cost_pair_type = NodeCostPair;

  const G *d;

  std::vector<node_id_type> _M_visited;
  std::vector<distance_type> _M_distance;
  std::vector<node_id_type> _M_predecessor;

public:
  explicit Labels (const G *d);

  // init for given query
  void init (node_id_type start_node, node_id_type target_node);

  bool reached (const node_id_type &node) const;
  const distance_t &distance (const node_id_type &node) const;
  const node_id_type &predecessor (const node_id_type &node) const;

  std::span<const node_id_type> all_visited () const;

  // label node with distance and predecessor
  void label (const node_cost_pair_type &node_cost_pair);
};



template <RoutableGraph G, typename N>
std::span<const typename Labels<G,N>::node_id_type>
Labels<G,N>::all_visited () const
{
  return std::span<const node_id_type> (_M_visited.begin (), _M_visited.end ());
}


template<RoutableGraph G, typename N>
Labels<G,N>::Labels (const G *d)
  : d (d), _M_predecessor (d->node_count (), NO_NODE_ID),
    _M_distance (d->node_count (), DISTANCE_INF)
{
  _M_visited.reserve(std::sqrt(d->node_count()));
}

template<RoutableGraph G, typename N>
void
Labels<G,N>::init (Labels<G,N>::node_id_type __start_node, Labels<G,N>::node_id_type __target_node)
{
  for (size_t index = 0; index < _M_visited.size (); ++index)
  {
    Labels<G,N>::node_id_type node = _M_visited[index];
    _M_predecessor[node] = NO_NODE_ID;
    _M_distance[node] = DISTANCE_INF;
  }

  _M_visited.clear ();
}

template<RoutableGraph Graph, typename N>
const Labels<Graph, N>::node_id_type &
Labels<Graph, N>::predecessor (const Labels<Graph, N>::node_id_type &node) const
{
  if (node == NO_NODE_ID) return NO_NODE_ID;
  return _M_predecessor[node];
}
template<RoutableGraph G, typename N>
const distance_t &
Labels<G, N>::distance (const Labels<G, N>::node_id_type &node) const
{
  if (node == NO_NODE_ID) return DISTANCE_INF;
  return _M_distance[node];
}

template<RoutableGraph G, typename N>
bool
Labels<G,N>::reached (const Labels<G,N>::node_id_type &node) const
{
  assert(node != NO_NODE_ID);
  return _M_distance[node] != DISTANCE_INF && _M_predecessor[node] != NO_NODE_ID;
}

template<RoutableGraph G, typename N>
void
Labels<G,N>::label (const Labels<G,N>::node_cost_pair_type &node_cost_pair)
{
  if (_M_predecessor[node_cost_pair.node] == NO_NODE_ID)
    _M_visited.push_back (node_cost_pair.node);
  _M_distance[node_cost_pair.node] = node_cost_pair.distance;
  _M_predecessor[node_cost_pair.node] = node_cost_pair.predecessor;
}
