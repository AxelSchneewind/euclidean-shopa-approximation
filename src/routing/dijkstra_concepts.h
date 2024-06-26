#pragma once

#include <concepts>
#include <vector>

template<typename T>
concept HasDistance = requires(T t) {
    t.distance();
};

template<typename T>
concept HasPredecessor = requires(T t){
    t.predecessor();
};

template<typename T>
concept HasNode = requires(T t){
    t.node();
};

template <typename T>
concept HasHeuristic = requires(T t) {
    t.heuristic();
};


template<typename T>
concept Topology = requires {
    typename T::node_id_type;
    typename T::edge_id_type;
} && requires(T t, typename T::node_id_type n) {
    t.outgoing_edges(n);
    t.incoming_edges(n);
    { t.edge_id(n, n) } -> std::convertible_to<typename T::edge_id_type>;
    { t.has_edge(n, n) } -> std::convertible_to<bool>;
} && requires(T t, typename T::edge_id_type e) {
    { t.source(e) } -> std::convertible_to<typename T::node_id_type>;
    { t.destination(e) } -> std::convertible_to<typename T::node_id_type>;
};

template <typename T>
concept NodeSet = requires {
    typename T::node_id_type;
} && requires(T t) {
    t.node_ids();
} && requires(T t, T::node_id_type i) {
    t.node(i);
};

template <typename T>
concept GeometricNodeSet = NodeSet<T>
&& requires {
    typename T::coordinate_type;
} && requires (T t, T::node_id_type i) {
    {t.node_coordinates(i)} -> std::convertible_to<typename T::coordinate_type>;
};

template<typename T>
concept EdgeSet = requires {
    typename T::node_id_type;
    typename T::edge_id_type;
} && requires(T t) {
    t.edge_ids();
};

template<typename T>
concept EdgeInfo = requires {
    typename T::node_id_type;
    typename T::edge_id_type;
    typename T::edge_info_type;
} && requires(T t, typename T::edge_id_type id) {
    { t.edge(id) } -> std::convertible_to<typename T::edge_info_type>;
} && requires (T t, T::node_id_type i) {
    { t.edge(i, i) } -> std::convertible_to<typename T::edge_info_type>;
};

template<typename T>
concept EdgeEnumerator = requires {
    typename T::node_id_type;
    typename T::edge_info_type;
} && requires(T t, typename T::node_id_type src) {
    { (*t.outgoing_edges(src).begin()).info } -> std::convertible_to<typename T::edge_info_type>;
    { (*t.outgoing_edges(src).begin()).destination } -> std::convertible_to<typename T::node_id_type>;
    ++t.outgoing_edges(src).begin();
    { (*t.incoming_edges(src).begin()).info } -> std::convertible_to<typename T::edge_info_type>;
    { (*t.incoming_edges(src).begin()).destination } -> std::convertible_to<typename T::node_id_type>;
    ++t.incoming_edges(src).begin();
};

template<typename G>
concept RoutableGraph = NodeSet<G> && Topology<G>;

template<typename Q, typename G>
concept DijkstraQueue = requires {
    typename Q::value_type;
} && requires(Q q, typename G::node_id_type n) {
    q.init(n, n);
} && requires(Q q) {
    q.pop();
    { q.top() } -> std::convertible_to<typename Q::value_type>;
    { q.empty() } -> std::convertible_to<bool>;
};

template <typename T, typename N>
concept HasInit = requires (T t, N n1, N n2) { t.init(n1, n2); };

template<typename L, typename NodeId, typename NodeCostPair, typename Label>
concept DijkstraLabels = requires(L l, NodeId node, NodeCostPair ncp, Label label) {
    l.at(node);
};


template<typename T, typename NodeCostPair>
concept NeighborsGetter = requires(T t, NodeCostPair n, std::vector<NodeCostPair> &r) { t(n, r); };
