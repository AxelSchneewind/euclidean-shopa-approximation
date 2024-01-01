#pragma once

#include "../graph/base_types.h"

#include <vector>

template<typename Graph, typename Labels>
struct steiner_neighbors {
private:
    Graph const &graph;
    Labels const &labels;

    double const max_angle;
    double const max_angle_cos;

    // buffer for coordinates
    std::vector<coordinate_t> destination_coordinates;

    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const &node_id = node.node;
        auto const &reached_from = node.node;
        auto &&base_node_id = graph.base_node_id(node.node);
        auto source_coordinate = graph.node(base_node_id).coordinates;

        coordinate_t const from_coordinate = graph.node(reached_from).coordinates;
        coordinate_t const direction = source_coordinate - from_coordinate;

        for (auto &&e: graph.base_graph().outgoing_edges(base_node_id)) [[likely]] {
            auto e_id = graph.base_graph().edge_id(base_node_id, e.destination);
            steiner_graph::node_id_type destination{e_id, 1};
            if (destination != node.predecessor) [[likely]] {
                out.emplace_back(destination, node.node, node.distance);
                destination_coordinates.emplace_back(graph.node(destination).coordinates);
            }
        }

        for (auto &&e: graph.base_graph().incoming_edges(base_node_id)) [[likely]] {
            auto e_id = graph.base_graph().edge_id(e.destination, base_node_id);          // TODO optimize
            steiner_graph::node_id_type destination(e_id, graph.steiner_info(e_id).node_count - 2U);
            if (destination != node.predecessor) [[likely]] {
                out.emplace_back(destination, node.node, node.distance);
                destination_coordinates.emplace_back(graph.node(destination).coordinates);
            }
        }

        // face-crossing edges
        auto &&triangle_edges = graph.base_polyhedron().node_edges(base_node_id);
        for (auto base_edge_id: triangle_edges) [[likely]] {
            auto &&destination_steiner_info = graph.steiner_info(base_edge_id);

            short i = 1;
            for (; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                steiner_graph::node_id_type const destination{base_edge_id, i};

                // TODO find out what breaks the labels reference here
                // if (labels.reached(destination)) [[likely]] continue;

                coordinate_t const destination_coordinate { graph.node(destination).coordinates };
                out.emplace_back(destination, node_id, node.distance);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }

        // compute distances (can be vectorized)
        for (unsigned int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].distance += distance(source_coordinate, destination_coordinates[e]);
        }
    }


    template<typename NodeCostPair>
    void from_steiner_node(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const &node_id = node.node;
        auto const &reached_from = node.predecessor;

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
        coordinate_t const source_coordinate = graph.node(node_id).coordinates;
        coordinate_t const from_coordinate = graph.node(reached_from).coordinates;
        coordinate_t const direction = source_coordinate - from_coordinate;
        auto &&steiner_info = graph.steiner_info(node_id.edge);

        // for neighboring node on own edge
        if (node_id.steiner_index < steiner_info.node_count - 1) [[likely]] {
            steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
            if (destination != reached_from) [[likely]] {
                coordinate_t destination_coordinate{graph.node(destination).coordinates};
                out.emplace_back(destination, node_id, node.distance);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }

        // for other neighboring node on own edge
        if (node_id.steiner_index > 0) [[likely]] {
            steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index - 1);
            if (destination != reached_from) [[likely]] {
                coordinate_t destination_coordinate{graph.node(destination).coordinates};
                out.emplace_back(destination, node_id, node.distance);
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

                auto &&destination_steiner_info = graph.steiner_info(base_edge_id);

                // binary search using the derivative over the angle depending on steiner index
                typename Graph::node_id_type::intra_edge_id_type l = 0;
                typename Graph::node_id_type::intra_edge_id_type r = destination_steiner_info.node_count - 2;
                typename Graph::node_id_type::intra_edge_id_type m = (r + l) / 2;
                double diff = 1.0;
                double cos = -1.0;
                while (l < r && diff != 0.0 && cos < max_angle_cos) {
                    steiner_graph::node_id_type const destination{base_edge_id, m};
                    coordinate_t const destination_coordinate = graph.node(destination).coordinates;
                    cos = angle_cos(direction, destination_coordinate - source_coordinate);

                    steiner_graph::node_id_type const destination_next(base_edge_id, m + 1);
                    coordinate_t const destination_coordinate_next = graph.node(destination_next).coordinates;
                    double cos_next = angle_cos(direction, destination_coordinate_next - source_coordinate);

                    diff = cos_next - cos;

                    l = (diff > 0.0) ? m + 1 : l;
                    r = (diff < 0.0) ? m - 1 : r;
                    m = (l + r) / 2;
                }

                coordinate_t last_direction = direction * -1;
                double spanner_angle_cos = std::cos(std::min(graph.epsilon() * M_PI_4, M_PI_4 / 16));
                for (auto j = m; j >= 0; --j) [[likely]] {
                    steiner_graph::node_id_type const destination(base_edge_id, j);
                    coordinate_t const destination_coordinate = graph.node(destination).coordinates;
                    coordinate_t const new_direction = destination_coordinate - source_coordinate;

                    if (angle_cos(direction, new_direction) < max_angle_cos) [[likely]]
                        break;
                    if (angle_cos(last_direction, new_direction) >= spanner_angle_cos) [[likely]]
                        continue;

                    out.emplace_back(destination, node_id, node.distance);
                    destination_coordinates.emplace_back(destination_coordinate);
                    last_direction = new_direction;
                }

                last_direction = direction * -1;
                for (auto j = m + 1; j < destination_steiner_info.node_count; ++j) [[likely]] {
                    steiner_graph::node_id_type const destination(base_edge_id, j);
                    coordinate_t const destination_coordinate = graph.node(destination).coordinates;
                    coordinate_t const new_direction = destination_coordinate - source_coordinate;

                    if (angle_cos(direction, destination_coordinate - source_coordinate) < max_angle_cos) [[likely]]
                        break;
                    if (angle_cos(last_direction, new_direction) >= spanner_angle_cos) [[likely]]
                        continue;

                    out.emplace_back(destination, node_id, node.distance);
                    destination_coordinates.emplace_back(destination_coordinate);
                    last_direction = new_direction;
                }
            }
        }

        // compute distances (can hopefully be vectorized)
        for (int e = 0; e < out.size(); ++e) [[likely]] {
            out[e].distance += distance(source_coordinate, destination_coordinates[e]);
        }
    }

public:
    steiner_neighbors(Graph const &graph, Labels const &labels)
            : graph(graph)
            , labels(labels)
            , max_angle{M_PI}// std::min(M_PI * graph.epsilon(), M_PI_2)} // 10º // std::atan(graph.epsilon() / 5)}
            , max_angle_cos{std::cos(max_angle)} {
    }

    template<typename... Args>
    steiner_neighbors(Graph const &graph, Labels const &labels, Args const &...) : steiner_neighbors(graph, labels) {}

    steiner_neighbors(steiner_neighbors&&) noexcept = default;
    steiner_neighbors& operator=(steiner_neighbors&&) noexcept = default;

    template<typename NodeCostPair>
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const &node_id = node.node;
        destination_coordinates.clear();

        assert(!is_none(node_id));

        if (graph.is_base_node(node_id)) [[unlikely]]
            return from_base_node(node, out);
        else
            return from_steiner_node(node, out);
    }
};
