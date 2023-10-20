#pragma once

#include "../util/remove_duplicates.h"
#include "unidirectional_adjacency_list.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

template <typename E>
const adjacency_list_edge<E> &
unidirectional_adjacency_list<E>::adjlist_edge (const edge_id_t &edge) const
{
  return edges[edge];
}

template <typename E>
void
unidirectional_adjacency_list<E>::adjacency_list_builder::add_node (const node_id_t &node)
{
  if (m_node_count <= node)
    m_node_count = node + 1;
}

template <typename E>
void
unidirectional_adjacency_list<E>::adjacency_list_builder::add_edge (const node_id_t &source,
								    const node_id_t &destination, const E &info)
{
  add_node (source);
  add_node (destination);

  auto edge = adjacency_list_edge<E> (source, destination, info);
  edges.push_back (edge);
  m_edge_count++;
}

template <typename E>
unidirectional_adjacency_list<E>
unidirectional_adjacency_list<E>::adjacency_list_builder::get () &
{
  m_edge_count = edges.size ();

  // order by source id
  auto order = [&] (adjacency_list_edge<E> e1, adjacency_list_edge<E> e2) { return e1.source < e2.source; };
  std::sort (edges.begin (), edges.end (), order);

  // remove duplicates
  remove_duplicates<adjacency_list_edge<E>>(edges);

  // make offset array
  int index = 0;
  offset.clear ();
  for (auto e : edges)
  {
    while (offset.size () <= e.source)
      offset.push_back (index);
    index++;
  }

  while (offset.size () <= m_node_count)
    offset.push_back (m_edge_count);

  return unidirectional_adjacency_list<E> (m_node_count, m_edge_count, std::move (offset), std::vector (edges));
}

template <typename E>
unidirectional_adjacency_list<E>
unidirectional_adjacency_list<E>::adjacency_list_builder::get () &&
{
  // order by source id
  auto order = [&] (adjacency_list_edge<E> e1, adjacency_list_edge<E> e2) { return e1.source < e2.source; };
  std::sort (edges.begin (), edges.end (), order);

  remove_duplicates<adjacency_list_edge<E>> (edges);
  m_edge_count = edges.size ();

  // make offset array
  int index = 0;
  offset.clear ();
  for (auto e : edges)
  {
    while (offset.size () <= e.source)
      offset.push_back (index);
    index++;
  }

  while (offset.size () <= m_node_count)
    offset.push_back (m_edge_count);

  return unidirectional_adjacency_list<E> (m_node_count, m_edge_count, std::move (offset), std::move (edges));
}

template <typename E>
inline size_t
unidirectional_adjacency_list<E>::node_count () const
{
  return nodecount;
}

template <typename E>
inline size_t
unidirectional_adjacency_list<E>::edge_count () const
{
  return edgecount;
}

template <typename E>
inline edge_id_t &
unidirectional_adjacency_list<E>::offset (const node_id_t &node)
{
  return offsets[node];
}

template <typename E>
inline edge_id_t &
unidirectional_adjacency_list<E>::offset_next (const node_id_t &node)
{
  return offsets[node + 1];
}

template <typename E>
inline const node_id_t &
unidirectional_adjacency_list<E>::source (const edge_id_t &edge) const
{
  return edges[edge].source;
}

template <typename E>
inline const node_id_t &
unidirectional_adjacency_list<E>::destination (const edge_id_t &edge) const
{
  return edges[edge].destination;
}

template <typename E>
inline edge_id_t
unidirectional_adjacency_list<E>::edge_index (const node_id_t &source, const node_id_t &dest) const
{
  edge_id_t idx = offsets[source];
  while (idx < offsets[source + 1] && destination (idx) != dest)
    idx++;

  return idx;
}

template <typename E>
inline bool
unidirectional_adjacency_list<E>::has_edge (const node_id_t &source, const node_id_t &dest) const
{
  edge_id_t idx = offsets[source];
  while (idx < offsets[source + 1] && destination (idx) != dest)
    idx++;

  return (idx < offsets[source + 1]);
}

template <typename E>
inline std::span<const adjacency_list_edge<E>, std::dynamic_extent>
unidirectional_adjacency_list<E>::node_edges (const node_id_t &node) const
{
  auto first = &edges[offset (node)];
  auto last = &edges[offset (node + 1)];
  return std::span (first, last);
}

template <typename E>
inline const edge_id_t &
unidirectional_adjacency_list<E>::offset (const node_id_t &node) const
{
  return offsets[node];
}

template <typename E>
inline const edge_id_t &
unidirectional_adjacency_list<E>::offset_next (const node_id_t &node) const
{
  return offsets[node + 1];
}

template <typename E>
inline const E &
unidirectional_adjacency_list<E>::edge (const edge_id_t &edge) const
{
  return edges[edge].info;
}

template <typename E> adjacency_list_edge<E>::adjacency_list_edge () = default;

template <typename E>
adjacency_list_edge<E>::adjacency_list_edge (node_id_t source, node_id_t destination, E info)
  : source (source), destination (destination), info (info)
{}

template <typename E>
adjacency_list_edge<E>
adjacency_list_edge<E>::invert (const std::vector<edge_id_t> &new_edge_indices) const
{
  return adjacency_list_edge (destination, source, info.invert (new_edge_indices));
}

template <typename E>
bool
adjacency_list_edge<E>::operator== (const adjacency_list_edge &other) const
{
  return (source == other.source && destination == other.destination); // TODO check info for equality
}

template <typename E> unidirectional_adjacency_list<E>::~unidirectional_adjacency_list ()
{
  offsets.clear ();
  offsets.shrink_to_fit ();
  edges.clear ();
  edges.shrink_to_fit ();
  offsets.emplace_back (0);
}

template <typename E>
unidirectional_adjacency_list<E>::unidirectional_adjacency_list (node_id_t node_count, edge_id_t edge_count)
  : nodecount (0), edgecount (0), offsets (), edges ()
{
  offsets.reserve (nodecount + 1);
  edges.reserve (edgecount);

  offsets.emplace_back (0);
}

template <typename E>
unidirectional_adjacency_list<E>::unidirectional_adjacency_list (node_id_t node_count, edge_id_t edge_count,
								 std::vector<edge_id_t> &&offsets,
								 std::vector<adjacency_list_edge<E>> &&edges)
  : nodecount (node_count), edgecount (edge_count), offsets (std::move (offsets)), edges (std::move (edges))
{
  if (this->offsets.size () < node_count) // given array is too small
    throw;
  else if (this->offsets.size () == node_count) // array is missing trailing offset entry
    this->offsets.push_back (edge_count);
  else
    this->offsets.back () = edge_count;
}

template <typename E>
unidirectional_adjacency_list<E>::unidirectional_adjacency_list (unidirectional_adjacency_list &&other) noexcept
  : nodecount (other.nodecount), edgecount (other.edgecount), offsets (std::move (other.offsets)),
    edges (std::move (other.edges))
{}

// access lists directly when not const
template <typename E>
std::vector<edge_id_t> &
unidirectional_adjacency_list<E>::offset_list ()
{
  return offsets;
}

template <typename E>
std::vector<adjacency_list_edge<E>> &
unidirectional_adjacency_list<E>::edge_list ()
{
  return edges;
}

// TODO refactor using adjacency_list_builder
// TODO fix
template <typename E>
unidirectional_adjacency_list<E>
unidirectional_adjacency_list<E>::inverse () const
{
  std::vector<std::vector<edge_id_t>> incoming_edges (node_count ());

  for (size_t edge_index = 0; edge_index < edge_count (); edge_index++)
  {
    const adjacency_list_edge<E> &edge = edges[edge_index];
    // insert nodes in inverse order
    incoming_edges[node_count () - edge.destination - 1].push_back (edge_index);
  }

  std::vector<edge_id_t> inv_offsets;
  inv_offsets.reserve (node_count ());
  std::vector<adjacency_list_edge<E>> inv_edges;
  inv_edges.reserve (edge_count ());

  // store new indices for each edge
  std::vector<edge_id_t> backward_ids;
  backward_ids.resize (edge_count ());
  // std::vector<edge_id_t>forward_ids;
  // forward_ids.resize (edge_count ());

  edge_id_t current = 0;
  for (size_t node_index = 0; node_index < node_count (); node_index++)
  {
    auto incoming = incoming_edges[node_count () - node_index - 1];
    for (edge_id_t edge : incoming)
    {
      // forward_ids[current] = edge;
      backward_ids[edge] = current++;
    }
  }

  for (size_t node_index = 0; node_index < node_count (); node_index++)
  {
    inv_offsets.push_back (inv_edges.size ());

    auto incoming = incoming_edges.back ();
    for (edge_id_t edge_id : incoming)
    {
      const adjacency_list_edge<E> &edge = edges[edge_id];
      inv_edges.push_back (edge.invert (backward_ids));
    }

    incoming_edges.pop_back ();
  }

  return unidirectional_adjacency_list (node_count (), edge_count (), std::move (inv_offsets), std::move (inv_edges));
}

template <typename E>
bool
unidirectional_adjacency_list<E>::operator== (const unidirectional_adjacency_list<E> &other)
{
  if (nodecount != other.node_count || edgecount != other.edge_count)
    return false;

  for (node_id_t i = 0; i < nodecount; i++)
    if (offsets[i] != other.offsets[i])
      return false;

  for (edge_id_t i = 0; i < edgecount; i++)
    if (edges[i] != other.edges[i])
      return false;

  return true;
}


template <template <typename> class A, typename E> struct std::hash<A<E>>;

template <typename E> struct std::hash<adjacency_list_edge<E>>
{
  std::size_t operator() (const adjacency_list_edge<E> &s) const noexcept
  {
    std::size_t h1 = std::hash<node_id_t>{}(s.source);
    std::size_t h2 = std::hash<node_id_t>{}(s.destination);
    return h1 ^ (h2 << 1);
  }
};