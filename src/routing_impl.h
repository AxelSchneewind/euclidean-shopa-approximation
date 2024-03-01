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
#include "triangulation/fast_map_impl.h"
#include "triangulation/steiner_neighbors_impl.h"
#include "triangulation/compact_node_info_container_impl.h"
#include "triangulation/steiner_graph_impl.h"
#include "triangulation/frontier_labels.h"
#include "triangulation/steiner_labels_impl.h"
#include "triangulation/polyhedron_impl.h"
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
                             _predecessor(optional::none_value<typename G::node_id_type>) {
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

struct distance_label {
public:
    using distance_type = steiner_graph::distance_type;
private:
    distance_type _distance;
public:

    static constexpr steiner_graph::node_id_type predecessor = optional::none_value<steiner_graph::node_id_type>;

    constexpr distance_label() : _distance(infinity<distance_type>) {}

    constexpr distance_label(distance_type dist) : _distance(dist) {}

    template<typename NCP>
    requires HasDistance<NCP>
    constexpr distance_label(NCP ncp) : _distance(ncp.distance()) {}

    constexpr distance_label(distance_type distance, steiner_graph::node_id_type) : _distance(distance) {}

    template<typename NCP>
    requires HasDistance<NCP>
    operator NCP() {
        return {
                optional::none_value<steiner_graph::node_id_type>,
                optional::none_value<steiner_graph::node_id_type>,
                _distance,
        };
    }

    distance_type& distance() { return _distance; }
    distance_type const& distance() const { return _distance; }
};


using std_graph_t = graph<node_t, edge_t, node_id_t, edge_id_t>;
using gl_graph_t = graph<node_t, gl_edge_t, node_id_t, edge_id_t>;
using ch_graph_t = graph<ch_node_t, ch_edge_t, node_id_t, edge_id_t>;
