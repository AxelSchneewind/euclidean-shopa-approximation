#pragma once

#ifndef DEBUG
#define NDEBUG
#endif
#include <cassert>

#include "graph/graph.h"
#include "routing/routing.h"

using std::vector;

typedef graph_t<node_t, edge_t> std_graph_t;
typedef routing_t<node_t, edge_t> std_routing_t;

typedef graph_t<ch_node_t, ch_edge_t> ch_graph_t;
typedef routing_t<ch_node_t, ch_edge_t> ch_routing_t;
