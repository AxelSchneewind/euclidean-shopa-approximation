#pragma once

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


struct a_star_info {
    // value from a* heuristic (distance + minimal remaining distance)
    distance_t value;
};
using a_star_node_cost_pair = node_cost_pair<std_graph_t, a_star_info>;

// check for struct packing
static_assert(sizeof(std_graph_t::node_info_type) == 2 * sizeof(distance_t));
static_assert(sizeof(std_graph_t::edge_info_type) == 1 * sizeof(distance_t));
static_assert(sizeof(ch_graph_t::node_info_type) == 2 * sizeof(distance_t) + 1 * sizeof(int));
static_assert(sizeof(ch_graph_t::edge_info_type) == 3 * sizeof(distance_t));
static_assert(sizeof(default_node_cost_pair) == 3 * sizeof(int));
static_assert(sizeof(a_star_node_cost_pair) == 4 * sizeof(int));


template <RoutableGraph G>
struct label_type {
    G::distance_type distance;
    G::node_id_type predecessor;
};
template <RoutableGraph G>
constexpr label_type<G> none_value() { return {none_value<G::distance_type>(), none_value<G::node_id_type>()}; }
template <>
constexpr label_type<steiner_graph> none_value() { return {none_value<steiner_graph::distance_type>(), none_value<steiner_graph::node_id_type>()}; }


using default_queue_t = dijkstra_queue<std_graph_t, default_node_cost_pair, Default<default_node_cost_pair>>;



using default_labels_t = node_labels<std_graph_t, label_type<std_graph_t>>;
using a_star_queue_t = a_star_queue<std_graph_t, a_star_node_cost_pair>;
using a_star_labels_t = node_labels<std_graph_t, label_type<std_graph_t>>;

using std_dijkstra = dijkstra<std_graph_t, default_queue_t, use_all_edges<std_graph_t>, default_labels_t>;
using a_star_dijkstra = dijkstra<std_graph_t, a_star_queue_t, use_all_edges<std_graph_t>, a_star_labels_t>;

using std_routing_t = router<std_graph_t, std_dijkstra>;
using a_star_routing_t = router<std_graph_t, a_star_dijkstra>;

using default_ch_labels_t = node_labels<ch_graph_t, default_node_cost_pair>;
using default_ch_queue_t = dijkstra_queue<ch_graph_t, default_node_cost_pair, Default<default_node_cost_pair>>;
using ch_routing_t = router<ch_graph_t, dijkstra<ch_graph_t, default_ch_queue_t, use_upward_edges<ch_graph_t>, default_ch_labels_t>>;


using steiner_a_star_node_cost_pair = node_cost_pair<steiner_graph, a_star_info>;
using steiner_queue_t = a_star_queue<steiner_graph, steiner_a_star_node_cost_pair>;
using steiner_labels_t = steiner_labels<steiner_graph, label_type<steiner_graph>>;
using steiner_dijkstra = dijkstra<steiner_graph, steiner_queue_t, use_all_edges<steiner_graph>, steiner_labels_t>;
using steiner_routing_t = router<steiner_graph, steiner_dijkstra>;


template<>
void
steiner_dijkstra::expand(steiner_dijkstra::node_cost_pair __node) {
    // static std::vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>> edges;
    // TODO
    fixed_capacity_vector<internal_adjacency_list_edge<steiner_graph::node_id_type, steiner_graph::edge_info_type>, 100> edges;
    assert(!is_none(__node.node));

    // get inverse edge for current base edge
    auto inv_edge = _M_graph->base_polyhedron().inverse_edge(__node.node.edge);

    // get triangles that have not been visited yet
    auto triangles = _M_graph->base_polyhedron().edge_faces(__node.node.edge);
    char triangle_first = 0;
    char triangle_last = 2;

    if (__node.predecessor.edge != __node.node.edge && __node.predecessor.edge != inv_edge) [[likely]] {
        auto visited_triangles = _M_graph->base_polyhedron().edge_faces(__node.predecessor.edge);

        if (triangles[0] == visited_triangles[0] || triangles[0] == visited_triangles[1] ||
            is_none(triangles[0])) [[unlikely]] {
            triangle_first = 1;
        }
        if (triangles[1] == visited_triangles[0] || triangles[1] == visited_triangles[1] ||
            is_none(triangles[1])) [[unlikely]] {
            triangle_last = 1;
        }
    }

    // make list of edges (i.e. destination/cost pairs)
    coordinate_t source_coordinate = _M_graph->node(__node.node).coordinates;
    for (char triangle_index = triangle_first; triangle_index < triangle_last; triangle_index++) [[unlikely]] {
        assert (!is_none(triangles[triangle_index]));
        auto triangle_edges = _M_graph->base_polyhedron().face_edges(triangles[triangle_index]);

        for (auto base_edge_id: triangle_edges) {
            auto steiner_info = _M_graph->steiner_info(base_edge_id);

            // for inverse edge
            if (base_edge_id == inv_edge && __node.node.steiner_index == steiner_info.node_count - 1) [[unlikely]] {
                steiner_graph::node_id_type destination = {inv_edge, _M_graph->steiner_info(inv_edge).node_count - 1};
                coordinate_t destination_coordinate = _M_graph->node(destination).coordinates;
                distance_t cost = distance(source_coordinate, destination_coordinate);
                edges.push_back({destination, {cost}});

                continue;
            }

            if (base_edge_id == __node.node.edge) [[unlikely]] {
                if (__node.node.steiner_index < steiner_info.node_count - 1) [[likely]] {
                    steiner_graph::node_id_type destination = {base_edge_id, __node.node.steiner_index + 1};
                    coordinate_t destination_coordinate = _M_graph->node(destination).coordinates;
                    distance_t cost = distance(source_coordinate, destination_coordinate);
                    edges.push_back({destination, {cost}});
                }

                if (__node.node.steiner_index > 0) [[likely]] {
                    steiner_graph::node_id_type destination = {base_edge_id, __node.node.steiner_index - 1};
                    coordinate_t destination_coordinate = _M_graph->node(destination).coordinates;
                    distance_t cost = distance(source_coordinate, destination_coordinate);
                    edges.push_back({destination, {cost}});
                }
                continue;
            }

            for (int i = 0; i < steiner_info.node_count; ++i) [[likely]] {
                steiner_graph::node_id_type destination = {base_edge_id, i};
                coordinate_t destination_coordinate = _M_graph->node(destination).coordinates;
                distance_t cost = distance(source_coordinate, destination_coordinate);
                edges.push_back({destination, {cost}});
            }
        }
    }

    assert(edges.size() <= 100);

    // TODO: vectorize cost calculations

    for (auto edge: edges) {
        // ignore certain edges
        if (!_M_use_edge(__node.node, edge)) [[unlikely]] {
            continue;
        }

        assert(!is_none(edge.destination));
        assert(_M_graph->has_edge(__node.node, edge.destination));

        const steiner_graph::node_id_type &successor = edge.destination;
        const distance_t successor_cost = _M_labels.get(successor).distance;
        const distance_t new_cost = _M_labels.get(__node.node).distance + edge.info.cost;

        if (new_cost < successor_cost) [[unlikely]] {
            // (re-)insert node into the queue with updated priority
            _M_queue.push(successor, __node.node, new_cost);
        }
    }

    edges.clear();
}
