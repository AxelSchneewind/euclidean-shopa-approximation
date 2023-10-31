#pragma once

#include <ranges>

template <typename G>
concept RoutableGraph = std::move_constructible<G> && std::copy_constructible<G> && requires {
  typename G::edge_info_type;
  typename G::node_id_type;
} && requires (G g, typename G::node_id_type n) {
  std::begin(g.node_edges(n));
  std::end(g.node_edges(n));
};

template <typename Q, typename G>
concept DijkstraQueue
  = std::move_constructible<Q> && std::copy_constructible<Q> && requires { typename Q::value_type; }
    && std::constructible_from<Q, const std::shared_ptr<const G> &>
    && requires (Q q, typename G::node_id_type n) { q.init (n, n); } && requires (Q q) {
	 q.pop ();
	 {
	   q.top ()
	 } -> std::convertible_to<typename Q::value_type>;
	 {
	   q.empty ()
	 } -> std::convertible_to<bool>;
       } && requires (Q q, typename G::node_id_type n, typename G::distance_type d) { q.push (n, n, d); };

template <typename L>
concept DijkstraLabels = std::move_constructible<L> && std::copy_constructible<L>;
