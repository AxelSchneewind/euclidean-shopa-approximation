#ifndef DEBUG
#define NDEBUG
#endif

#include "file-io/fmi_file_io.h"
#include "file-io/formatters.h"
#include "file-io/gl_file_io.h"
#include "file-io/triangulation_file_io.h"
#include "graph/adjacency_list.h"
#include "graph/base_types.h"
#include "graph/graph.h"
#include "graph/unidirectional_adjacency_list.h"
#include "routing/dijkstra.h"
#include "routing/router.h"

#include "file-io/fmi_file_io_impl.h"
#include "file-io/formatters_impl.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io_impl.h"
#include "graph/adjacency_list_impl.h"
#include "graph/base_types_impl.h"
#include "graph/graph_impl.h"
#include "graph/unidirectional_adjacency_list_impl.h"
#include "routing/dijkstra_impl.h"
#include "routing/router_impl.h"

using std_graph_t = graph<node_t, edge_t>;
using ch_graph_t = graph<ch_node_t, ch_edge_t>;

using default_node_cost_pair = node_cost_pair<std::nullptr_t>;
struct a_star_info
{
  distance_t remaining;
};
using a_star_node_cost_pair = node_cost_pair<a_star_info>;

// check for struct packing
static_assert (sizeof (std_graph_t::node_info_type) == 2 * sizeof (distance_t));
static_assert (sizeof (std_graph_t::edge_info_type) == 1 * sizeof (distance_t));
static_assert (sizeof (ch_graph_t::node_info_type) == 2 * sizeof (distance_t) + 1 * sizeof (int));
static_assert (sizeof (ch_graph_t::edge_info_type) == 3 * sizeof (distance_t));
static_assert (sizeof (default_node_cost_pair) == 3 * sizeof (int));
static_assert (sizeof (a_star_node_cost_pair) == 4 * sizeof (int));

using default_queue_t = dijkstra_queue<std_graph_t, default_node_cost_pair, Default<default_node_cost_pair>>;
using a_star_queue_t = a_star_queue<std_graph_t, a_star_node_cost_pair>;

using std_routing_t = routing<std_graph_t, dijkstra<std_graph_t, default_queue_t, use_all_edges<std_graph_t>>>;
using a_star_routing_t = routing<std_graph_t, dijkstra<std_graph_t, a_star_queue_t, use_all_edges<std_graph_t>>>;

using default_ch_queue_t = dijkstra_queue<ch_graph_t, default_node_cost_pair, Default<default_node_cost_pair>>;
using ch_routing_t = routing<ch_graph_t, dijkstra<ch_graph_t, default_ch_queue_t, use_upward_edges<ch_graph_t>>>;
