#pragma once

#include "file-io/fmi_file_io_impl.h"
#include "file-io/formatters_impl.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io_impl.h"
#include "graph/adjacency_list_impl.h"
#include "graph/base_types_impl.h"
#include "graph/graph_impl.h"
#include "graph/unidirectional_adjacency_list_impl.h"
#include "routing/dijkstra_queues.h"
#include "routing/dijkstra_impl.h"
#include "routing/default_neighbors_impl.h"
#include "routing/router_impl.h"
#include "triangulation/fast_map_impl.h"
#include "triangulation/steiner_neighbors_impl.h"
#include "triangulation/compact_node_info_container_impl.h"
#include "triangulation/steiner_graph_impl.h"
#include "triangulation/frontier_labels.h"
#include "triangulation/steiner_labels_impl.h"
#include "triangulation/polyhedron_impl.h"
#include "routing/node_labels_impl.h"

#include "graph/geometry_impl.h"

using std_graph_t = graph<node_t, edge_t, node_id_t, edge_id_t>;
using gl_graph_t = graph<node_t, gl_edge_t, node_id_t, edge_id_t>;
using ch_graph_t = graph<ch_node_t, ch_edge_t, node_id_t, edge_id_t>;

// just to have the sizes somewhere and see when they change
static_assert(steiner_graph::SIZE_PER_NODE == 24);
static_assert(steiner_graph::SIZE_PER_EDGE == 64);

static_assert(std_graph_t::SIZE_PER_NODE == 24);
static_assert(std_graph_t::SIZE_PER_EDGE == 40);
