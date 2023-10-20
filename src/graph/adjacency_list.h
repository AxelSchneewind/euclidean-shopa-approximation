#pragma once

#include "unidirectional_adjacency_list.h"
#include <memory>

template <typename E> class adjacency_list
{
private:
  adjacency_list (std::shared_ptr<const unidirectional_adjacency_list<E>> __forward,
		  std::shared_ptr<unidirectional_adjacency_list<E>> __backward);

  adjacency_list (const std::shared_ptr<const unidirectional_adjacency_list<E>> &__forward,
		  const std::shared_ptr<const unidirectional_adjacency_list<E>> &__backward)
    : m_forward (__forward), m_backward (__backward){};

  std::shared_ptr<const unidirectional_adjacency_list<E>> m_forward;
  std::shared_ptr<const unidirectional_adjacency_list<E>> m_backward;

public:
  static adjacency_list<E>
  make_bidirectional (const std::shared_ptr<const unidirectional_adjacency_list<E>> &__forward);

  static adjacency_list<E> make_bidirectional (unidirectional_adjacency_list<E> &&__forward);

  static adjacency_list<E>
  make_bidirectional_undirected (const std::shared_ptr<const unidirectional_adjacency_list<E>> &__edges);

  static adjacency_list<E> make_bidirectional_undirected (unidirectional_adjacency_list<E> &&__forward);

  adjacency_list (const adjacency_list<E> &__other) noexcept
    : m_forward (__other.m_forward), m_backward (__other.m_backward)
  {}

  adjacency_list (adjacency_list<E> &&__other) noexcept
    : m_forward (std::move (__other.m_forward)), m_backward (std::move (__other.m_backward))
  {}

  adjacency_list<E> &operator= (const adjacency_list<E> &other) = default;
  adjacency_list<E> &operator= (adjacency_list<E> &&other) = default;

  adjacency_list inverse () const;

  [[nodiscard]] inline size_t node_count () const;

  [[nodiscard]] inline size_t edge_count () const;

  const unidirectional_adjacency_list<E> &forward () const { return *m_forward; }

  const unidirectional_adjacency_list<E> &backward () const { return *m_backward; }

  bool operator== (const adjacency_list<E> &__other);
};