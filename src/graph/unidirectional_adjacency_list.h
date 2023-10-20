#pragma once

#include "base_types.h"

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>


template <typename E> struct adjacency_list_edge
{

  node_id_t source;
  node_id_t destination;
  E info;

  adjacency_list_edge ();

  adjacency_list_edge (node_id_t source, node_id_t destination, E info);

  adjacency_list_edge invert (const std::vector<edge_id_t> &new_edge_indices) const;

  bool operator== (const adjacency_list_edge &other) const;
};

// class that stores edges (topology) of a m_adj_list
template <typename E> class unidirectional_adjacency_list
{
public:
  class adjacency_list_builder
  {
  private:
    size_t m_node_count;
    size_t m_edge_count;

    std::vector<edge_id_t> offset;
    std::vector<adjacency_list_edge<E> > edges;

  public:
    adjacency_list_builder () : m_node_count (0), m_edge_count (0){};

    void add_node (const node_id_t &node);

    void add_edge (const adjacency_list_edge<E> &edge) { edges.push_back (edge); };

    void add_edge (const node_id_t &source, const node_id_t &destination, const E &info);

    unidirectional_adjacency_list<E> get () &;
    unidirectional_adjacency_list<E> get () &&;
  };

private:
  size_t nodecount;
  size_t edgecount;
  std::vector<edge_id_t> offsets;
  std::vector<adjacency_list_edge<E> > edges;

public:
  ~unidirectional_adjacency_list ();

  unidirectional_adjacency_list (node_id_t node_count, edge_id_t edge_count);

  unidirectional_adjacency_list (node_id_t node_count, edge_id_t edge_count, std::vector<edge_id_t> &&offsets,
				 std::vector<adjacency_list_edge<E> > &&edges);

  // move constructor
  unidirectional_adjacency_list (unidirectional_adjacency_list &&other) noexcept;

  // unidirectional_adjacency_list<E> inverse() const;
  unidirectional_adjacency_list<E> inverse () const;

  // access lists directly when not const
  std::vector<edge_id_t> &offset_list ();

  std::vector<adjacency_list_edge<E> > &edge_list ();

  inline size_t node_count () const;

  inline size_t edge_count () const;

  inline edge_id_t &offset (const node_id_t &node);

  inline edge_id_t &offset_next (const node_id_t &node);

  inline const node_id_t &source (const edge_id_t &edge) const;

  inline const node_id_t &destination (const edge_id_t &edge) const;

  inline const E &edge (const edge_id_t &edge) const;

  inline const adjacency_list_edge<E> &adjlist_edge (const edge_id_t &edge) const;

  inline edge_id_t edge_index (const node_id_t &source, const node_id_t &dest) const;

  inline bool has_edge (const node_id_t &source, const node_id_t &dest) const;

  inline std::span<const adjacency_list_edge<E>, std::dynamic_extent> node_edges (const node_id_t &node) const;

  inline const edge_id_t &offset (const node_id_t &node) const;

  inline const edge_id_t &offset_next (const node_id_t &node) const;

  bool operator== (const unidirectional_adjacency_list<E> &other);
};

template<typename E>
class std::hash<adjacency_list_edge<E>>;