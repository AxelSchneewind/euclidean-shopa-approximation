#pragma once

#include "../graph/base_types.h"

#include <vector>

template<typename Graph>
struct steiner_neighbors {
private:
    Graph const &graph;

    // buffer for coordinates
    std::vector<coordinate_t> destination_coordinates;

    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto base_node_id = graph.base_node_id(node.node);

        // NOTE: paper does not require these edges, but as bending only occurs at base nodes
        // these edges allow for less outgoing edges per steiner node
        auto &&reachable_edges = graph.base_polyhedron().node_edges(base_node_id);
        for (auto &&e: reachable_edges) [[likely]] {
            auto destination_steiner_info = graph.steiner_info(e);

            for (typename Graph::node_id_type::intra_edge_id_type i = 1; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                steiner_graph::node_id_type destination {e, i};
                destination_coordinates.emplace_back(graph.node(destination).coordinates);
                out.emplace_back(destination, node.node, 0);
            }
        }

        for (auto &&e: graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
            auto e_id = graph.base_graph().edge_id(base_node_id, e.destination);
            steiner_graph::node_id_type destination {e_id, 1};
            out.emplace_back(destination, node.node, 0);
            destination_coordinates.emplace_back(graph.node(destination).coordinates);
        }

        for (auto &&e: graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
            auto e_id = graph.base_graph().edge_id(e.destination, base_node_id);          // TODO optimize
            steiner_graph::node_id_type destination {e_id, graph.steiner_info(e_id).node_count - 2U};
            if (destination != node.predecessor) [[likely]] {
                out.emplace_back(destination, node.node, 0);
                destination_coordinates.emplace_back(graph.node(destination).coordinates);
            }
        }

        // compute distances (can be vectorized)
        coordinate_t const source_coordinate = graph.node(base_node_id).coordinates;
        for (unsigned int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].distance = distance(source_coordinate, destination_coordinates[e]) + node.distance;
        }
    }


public:
    steiner_neighbors(Graph const &graph) : graph(graph) {}

    template<typename... Args>
    steiner_neighbors(Graph const &graph, Args const&...) : graph(graph) {}

    // TODO: prune according to paper section 2.4
    template<typename NodeCostPair>
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const& node_id = node.node;
        auto const& reached_from = node.predecessor;
        float constexpr max_angle = M_PI_4;// ~10º // std::atan(3.0 * graph.epsilon());

        destination_coordinates.clear();

        assert(!is_none(node_id));

        if (graph.is_base_node(node_id)) [[unlikely]]
            return from_base_node(node, out);

        // get triangles that have not been visited yet
        auto &&triangles = graph.base_polyhedron().edge_faces(node_id.edge);
        unsigned char triangle_first = 0;
        unsigned char triangle_last = 2;

        // if this node is reached via a face crossing segment, only use edges in next face
        if (reached_from.edge != node_id.edge) [[likely]] {
            auto visited_triangles = graph.base_polyhedron().edge_faces(reached_from.edge);

            if (is_none(triangles[0]) || triangles[0] == visited_triangles[0] || triangles[0] == visited_triangles[1])
                    [[unlikely]] {
                triangle_first = 1;
            }
            if (is_none(triangles[1]) || triangles[1] == visited_triangles[0] || triangles[1] == visited_triangles[1])
                    [[unlikely]] {
                triangle_last = 1;
            }
        } else {
            if (is_none(triangles[0])) [[unlikely]] {
                triangle_first = 1;
            }
            if (is_none(triangles[1])) [[unlikely]] {
                triangle_last = 1;
            }
        }

        // make list of edges (i.e. destination/cost pairs)
        coordinate_t source_coordinate = graph.node(node_id).coordinates;
        coordinate_t from_coordinate = graph.node(reached_from).coordinates;
        const auto &&steiner_info = graph.steiner_info(node_id.edge);

        // for neighboring node on own edge
        if (node_id.steiner_index < steiner_info.node_count - 1) [[likely]] {
            steiner_graph::node_id_type const destination{node_id.edge, node_id.steiner_index + 1};
            if (destination != reached_from) [[likely]] {
                coordinate_t destination_coordinate { graph.node(destination).coordinates };
                out.emplace_back(destination, node_id, 0);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }

        // for other neighboring node on own edge
        if (node_id.steiner_index > 0) [[likely]] {
            steiner_graph::node_id_type const destination{node_id.edge, node_id.steiner_index - 1};
            if (destination != reached_from) [[likely]]{
                coordinate_t destination_coordinate { graph.node(destination).coordinates };
                out.emplace_back(destination, node_id, 0);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }

        // face-crossing edges
        for (unsigned char triangle_index = triangle_first;
             triangle_index < triangle_last; triangle_index++) [[unlikely]] {
            assert(!is_none(triangles[triangle_index]));
            auto triangle_edges = graph.base_polyhedron().face_edges(triangles[triangle_index]);

            for (auto base_edge_id: triangle_edges) [[likely]] {
                if (base_edge_id == node_id.edge) [[unlikely]]
                    continue;

                const auto &&destination_steiner_info = graph.steiner_info(base_edge_id);

                // linear search
                typename Graph::node_id_type::intra_edge_id_type i = 1;
                for (; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                    steiner_graph::node_id_type const destination {base_edge_id, i};
                    coordinate_t const destination_coordinate = graph.node(destination).coordinates;
                    if (angle(from_coordinate, source_coordinate, source_coordinate, destination_coordinate) <
                        max_angle) [[unlikely]]
                        break;
                }

                for (; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                    steiner_graph::node_id_type const destination {base_edge_id, i};
                    coordinate_t destination_coordinate = graph.node(destination).coordinates;

                    if (angle(from_coordinate, source_coordinate, source_coordinate, destination_coordinate) >
                        max_angle) [[likely]]
                        break;

                    out.emplace_back(destination, node_id, 0);
                    destination_coordinates.emplace_back(destination_coordinate);
                }
            }
        }

        // compute distances (can be vectorized)
        for (unsigned int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].distance = distance(source_coordinate, destination_coordinates[e]) + node.distance;
        }
    }
};
