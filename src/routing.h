
#include "file-io/fmi_file_io.h"
#include "file-io/formatters.h"
#include "file-io/gl_file_io.h"
#include "file-io/triangulation_file_io.h"
#include "graph/adjacency_list.h"
#include "graph/base_types.h"
#include "graph/graph.h"
#include "graph/unidirectional_adjacency_list.h"
#include "routing/dijkstra.h"
#include "routing/node_labels.h"
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
#include "triangulation/steiner_graph_impl.h"
#include "triangulation/steiner_labels.h"

using std_graph_t = graph<node_t, edge_t, node_id_t, edge_id_t>;
using ch_graph_t = graph<ch_node_t, ch_edge_t, node_id_t, edge_id_t>;

using default_node_cost_pair = node_cost_pair<std_graph_t, std::nullptr_t>;
struct a_star_info
{
  // value from a* heuristic (distance + minimal remaining distance)
  distance_t value;
};
using a_star_node_cost_pair = node_cost_pair<std_graph_t, a_star_info>;

// check for struct packing
static_assert (sizeof (std_graph_t::node_info_type) == 2 * sizeof (distance_t));
static_assert (sizeof (std_graph_t::edge_info_type) == 1 * sizeof (distance_t));
static_assert (sizeof (ch_graph_t::node_info_type) == 2 * sizeof (distance_t) + 1 * sizeof (int));
static_assert (sizeof (ch_graph_t::edge_info_type) == 3 * sizeof (distance_t));
static_assert (sizeof (default_node_cost_pair) == 3 * sizeof (int));
static_assert (sizeof (a_star_node_cost_pair) == 4 * sizeof (int));

using default_queue_t = dijkstra_queue<std_graph_t, default_node_cost_pair, Default<default_node_cost_pair>>;
using default_labels_t = node_labels<std_graph_t, default_node_cost_pair>;
using a_star_queue_t = a_star_queue<std_graph_t, a_star_node_cost_pair>;
using a_star_labels_t = node_labels<std_graph_t, a_star_node_cost_pair>;

using std_dijkstra = dijkstra<std_graph_t, default_queue_t, use_all_edges<std_graph_t>, default_labels_t>;
using a_star_dijkstra = dijkstra<std_graph_t, a_star_queue_t, use_all_edges<std_graph_t>, a_star_labels_t>;

using std_routing_t = router<std_graph_t, std_dijkstra>;
using a_star_routing_t = router<std_graph_t, a_star_dijkstra>;

using default_ch_labels_t = node_labels<ch_graph_t, default_node_cost_pair>;
using default_ch_queue_t = dijkstra_queue<ch_graph_t, default_node_cost_pair, Default<default_node_cost_pair>>;
using ch_routing_t = router<ch_graph_t, dijkstra<ch_graph_t, default_ch_queue_t, use_upward_edges<ch_graph_t>, default_ch_labels_t>>;


using steiner_queue_t = dijkstra_queue<steiner_graph, node_cost_pair<steiner_graph>, Default<node_cost_pair<steiner_graph>>>;
using steiner_labels_t = steiner_labels<steiner_graph, node_cost_pair<steiner_graph>>;
using steiner_dijkstra = dijkstra<steiner_graph, steiner_queue_t, use_all_edges<steiner_graph>, steiner_labels_t>;
using steiner_routing_t = router<steiner_graph, steiner_dijkstra>;
