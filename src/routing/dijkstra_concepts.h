#pragma once

#include <ranges>
#include <memory>



template<typename T>
concept Topology =requires {
    typename T::node_id_type;
    typename T::edge_id_type;
} && requires(T t, typename T::node_id_type n) {
    t.outgoing_edges(n);
    t.incoming_edges(n);
    { t.edge_id(n, n) } -> std::convertible_to<typename T::edge_id_type>;
};


template<typename G>
concept RoutableGraph = std::move_constructible<G> && std::copy_constructible<G> && requires {
    typename G::node_id_type;
    typename G::edge_id_type;

    typename G::node_info_type;
    typename G::edge_info_type;

    typename G::topology_type;
}
                        && requires(G g) {
    { g.topology() } -> std::convertible_to<typename G::topology_type>;           // TODO check that Topology concept is fulfilled
    { g.inverse_topology() } -> std::convertible_to<typename G::topology_type>;
} && requires(G g) {
    g.node_ids();
};


template<typename Q, typename G>
concept DijkstraQueue
= std::move_constructible<Q> && std::copy_constructible<Q> && requires { typename Q::value_type; }
  && std::constructible_from<Q, const std::shared_ptr<const G> &>
  && requires(Q q, typename G::node_id_type n) { q.init(n, n); } && requires(Q q) {
    q.pop();
    {
    q.top()
    } -> std::convertible_to<typename Q::value_type>;
    {
    q.empty()
    } -> std::convertible_to<bool>;
} && requires(Q q, typename G::node_id_type n, typename G::distance_type d) { q.push(n, n, d); };

template<typename L>
concept DijkstraLabels = std::move_constructible<L> && std::copy_constructible<L>;
