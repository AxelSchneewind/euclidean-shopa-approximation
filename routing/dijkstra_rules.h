#pragma once

#include "dijkstra.h"
#include <tuple>

template<typename Dijkstra>
bool
cost_ascending(const typename Dijkstra::node_cost_pair &n1, const typename Dijkstra::node_cost_pair &n2, const Dijkstra& dijkstra) {
    return n1.distance > n2.distance;
}

template<typename Dijkstra>
bool
a_star_ascending(const typename Dijkstra::node_cost_pair &n1, const typename Dijkstra::node_cost_pair &n2, const Dijkstra& dijkstra) {
    return n1.distance > n2.distance;
}
