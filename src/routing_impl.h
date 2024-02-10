#pragma once

#include "file-io/fmi_file_io_impl.h"
#include "file-io/formatters_impl.h"
#include "file-io/gl_file_io_impl.h"
#include "file-io/triangulation_file_io_impl.h"
#include "graph/adjacency_list_impl.h"
#include "graph/base_types_impl.h"
#include "graph/graph_impl.h"
#include "graph/unidirectional_adjacency_list_impl.h"
#include "routing/node_cost_pair.h"
#include "routing/dijkstra_queues.h"
#include "routing/a_star_queue.h"
#include "routing/dijkstra_impl.h"
#include "routing/default_neighbors_impl.h"
#include "routing/router_impl.h"
#include "routing/router_bidirectional_impl.h"
#include "triangulation/steiner_neighbors_impl.h"
#include "triangulation/pruned_steiner_neighbors_impl.h"
#include "triangulation/node_info_container_impl.h"
#include "triangulation/node_info_array_impl.h"
#include "triangulation/compact_node_info_container_impl.h"
#include "triangulation/steiner_graph_impl.h"
#include "triangulation/frontier_labels.h"
#include "triangulation/steiner_labels_impl.h"
#include "routing/node_labels_impl.h"

#include "graph/geometry_impl.h"

template<RoutableGraph G>
struct label_type {
private:
    typename G::distance_type _distance;
    typename G::node_id_type _predecessor;
public:
    constexpr label_type(G::distance_type distance, G::node_id_type predecessor) : _distance(distance),
                                                                                   _predecessor(predecessor) {
    };

    constexpr label_type() : _distance(infinity<typename G::distance_type>),
                             _predecessor(none_value<typename G::node_id_type>) {
    };

    // TODO remove these constructors and instead use a template parameter for conversion in node_labels classes
    template<typename NCP>
    requires HasDistance<NCP> && HasPredecessor<NCP>
    label_type(NCP const &ncp) : _distance{ncp.distance()}, _predecessor{ncp.predecessor()} {};

    // enable this if the resulting tree should link each edge with the predecessor that set its direction
    // template<typename NCP>
    // requires HasDistance<NCP> && HasPredecessor<NCP> && HasFaceCrossingPredecessor<NCP, G>
    // label_type(NCP const &ncp) : _distance{ncp.distance()}, _predecessor{ncp.face_crossing_predecessor()} {};

    template<typename NCP>
    requires HasDistance<NCP> && HasPredecessor<NCP>
    label_type &operator=(NCP const &ncp) {
        _distance = ncp.distance();
        _predecessor = ncp.predecessor();
        return *this;
    };

    typename G::distance_type &distance() { return _distance; };

    typename G::node_id_type &predecessor() { return _predecessor; };

    typename G::distance_type const& distance() const { return _distance; };

    typename G::node_id_type const& predecessor() const { return _predecessor; };
};

template<RoutableGraph Graph>
constexpr label_type<Graph>
//none_value<label_type<Graph>> = {infinity<typename Graph::distance_type>, none_value<typename Graph::node_id_type>};
none_value<label_type<Graph>> = {};


using std_graph_t = graph<node_t, edge_t, node_id_t, edge_id_t>;
using gl_graph_t = graph<node_t, gl_edge_t, node_id_t, edge_id_t>;
using ch_graph_t = graph<ch_node_t, ch_edge_t, node_id_t, edge_id_t>;

using default_node_cost_pair = node_cost_pair<std_graph_t::node_id_type, std_graph_t::distance_type>;

using a_star_node_cost_pair = node_cost_pair<std_graph_t::node_id_type, std_graph_t::distance_type, a_star_info>;


using default_queue_t = dijkstra_queue<std_graph_t, default_node_cost_pair, Default>;


using default_labels_t = node_labels<std_graph_t, label_type<std_graph_t>>;
using a_star_queue_t = a_star_queue<std_graph_t, a_star_node_cost_pair>;
using a_star_labels_t = node_labels<std_graph_t, label_type<std_graph_t>>;

using std_dijkstra = dijkstra<std_graph_t, default_queue_t, default_labels_t, default_neighbors<std_graph_t>,
        use_all_edges<std_graph_t>>;
using a_star_dijkstra = dijkstra<std_graph_t, a_star_queue_t, a_star_labels_t, default_neighbors<std_graph_t>,
        use_all_edges<std_graph_t>>;

using std_routing_t = bidirectional_router<std_graph_t, std_dijkstra>;
using a_star_routing_t = bidirectional_router<std_graph_t, a_star_dijkstra>;

using default_ch_labels_t = node_labels<ch_graph_t, label_type<ch_graph_t>>;
using default_ch_queue_t = dijkstra_queue<ch_graph_t, default_node_cost_pair, Default>;
using ch_dijkstra = dijkstra<ch_graph_t, default_ch_queue_t, default_ch_labels_t, default_neighbors<ch_graph_t>,
        use_upward_edges<ch_graph_t>>;
using ch_routing_t = router<ch_graph_t, ch_dijkstra>;


using steiner_a_star_node_cost_pair = node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type,
        a_star_info>;
using steiner_node_cost_pair = node_cost_pair<steiner_graph::node_id_type, steiner_graph::distance_type, void>;

using steiner_queue_t = dijkstra_queue<steiner_graph, steiner_node_cost_pair>;
using steiner_labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>;
using steiner_dijkstra = dijkstra<steiner_graph, steiner_queue_t, steiner_labels_t, steiner_neighbors<steiner_graph,
        steiner_labels_t>, use_all_edges<steiner_graph>>;
using steiner_routing_t = router<steiner_graph, steiner_dijkstra>;
