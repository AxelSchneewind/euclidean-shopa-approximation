#pragma once

#include "../graph/base_types.h"

#include <vector>

template<typename Graph, typename Labels>
struct pruned_steiner_neighbors {
private:
    Graph const &graph;
    Labels const& labels;

    coordinate_t source_coordinate;
    coordinate_t predecessor_coordinate;

    static constexpr float max_angle = M_PI_4 / 2;

    std::vector<coordinate_t> destination_coordinates;

    template<typename NodeCostPair>
    void on_edge_neighbors(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const& node_id = node.node;
        auto const& reached_from = node.predecessor;
        auto const& _steiner_info = graph.steiner_info(node_id.edge);

        // for neighboring node on own edge
        if (node_id.steiner_index < _steiner_info.node_count - 1) [[likely]] {
            steiner_graph::node_id_type const destination(node_id.edge, node_id.steiner_index + 1);
            if (destination != reached_from) [[likely]] {
                coordinate_t destination_coordinate = graph.node(destination).coordinates;
                out.emplace_back(destination, node_id, 0);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }

        // for other neighboring node on own edge
        if (node_id.steiner_index > 0) [[likely]] {
            steiner_graph::node_id_type const destination(node_id.edge, 0);
            if (destination != reached_from) [[likely]]{
                coordinate_t destination_coordinate = graph.node(destination).coordinates;
                out.emplace_back(destination, node_id, 0);
                destination_coordinates.emplace_back(destination_coordinate);
            }
        }
    }


    template<typename NodeCostPair>
    void from_base_node(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const& __base_node_id = graph.base_node_id(node.node);

        for (auto &&e: graph.base_graph().outgoing_edges(__base_node_id)) [[likely]] {
            auto e_id = graph.base_graph().edge_id(__base_node_id, e.destination);
            steiner_graph::node_id_type destination = {e_id, 1};
            out.emplace_back(destination, node.node, 0);
            destination_coordinates.emplace_back(graph.node(destination).coordinates);
        }

        for (auto &&e: graph.base_graph().incoming_edges(__base_node_id)) [[likely]] {
            auto e_id = graph.base_graph().edge_id(e.destination, __base_node_id);
            steiner_graph::node_id_type destination = {e_id, graph.steiner_info(e_id).node_count - 2U};
            if (destination != node.predecessor) [[likely]] {
                out.emplace_back(destination, node.node, 0);
                destination_coordinates.emplace_back(graph.node(destination).coordinates);
            }
        }
    }

    template<typename NodeCostPair>
    void from_base_neighboring_node(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const& node_id = node.node;
        auto const& reached_from = node.predecessor;
        const auto &&_steiner_info = graph.steiner_info(node_id.edge);

        // for angle computations
        coordinate_t source_coordinate = graph.node(node_id).coordinates;
        coordinate_t from_coordinate = graph.node(reached_from).coordinates;

        // face-crossing edges
        auto&& triangles = graph.base_polyhedron().edge_faces(node_id.edge);
        for (unsigned char triangle_index = 0; triangle_index < 2; triangle_index++) [[unlikely]] {
            if(is_none(triangles[triangle_index])) break;

            auto triangle_edges = graph.base_polyhedron().face_edges(triangles[triangle_index]);
            for (auto base_edge_id: triangle_edges) [[likely]] {
                if (base_edge_id == node_id.edge) [[unlikely]]
                    continue;

                const auto &&destination_steiner_info = graph.steiner_info(base_edge_id);

                for (unsigned short i = 0; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                    steiner_graph::node_id_type const destination = {base_edge_id, i};
                    coordinate_t destination_coordinate = graph.node(destination).coordinates;

                    // if (angle(from_coordinate, source_coordinate, source_coordinate, destination_coordinate) < max_angle)
                        // continue;

                    out.emplace_back(destination, node_id, 0);
                    destination_coordinates.emplace_back(destination_coordinate);
                }
            }
        }
    }


    template<typename NodeCostPair>
    void from_edge_using_segment(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const& node_id = node.node;
        auto reached_from_on_edge = node.predecessor;
        auto reached_from_other_edge = labels.get(node.predecessor).predecessor;
        const auto &&_steiner_info = graph.steiner_info(node_id.edge);

        // add neighboring steiner points
        on_edge_neighbors(node, out);

        // get last predecessor that is not on this edge
        while(reached_from_other_edge.edge != node_id.edge && reached_from_other_edge != labels.get(reached_from_other_edge).predecessor) [[unlikely]] {
            reached_from_on_edge = reached_from_other_edge;
            reached_from_other_edge = labels.get(reached_from_other_edge).predecessor;
        }

        // for angle computations
        coordinate_t from_coordinate = graph.node(reached_from_on_edge).coordinates;
        coordinate_t from_other_edge_coordinate = graph.node(reached_from_other_edge).coordinates;

        // face-crossing edges
        auto&& triangles = graph.base_polyhedron().edge_faces(node_id.edge);
        for (unsigned char triangle_index = 0; triangle_index < 2; triangle_index++) [[unlikely]] {
            if(is_none(triangles[triangle_index])) break;

            auto triangle_edges = graph.base_polyhedron().face_edges(triangles[triangle_index]);
            for (auto base_edge_id: triangle_edges) [[likely]] {
                if (base_edge_id == node_id.edge) [[unlikely]]
                    continue;

                const auto &&destination_steiner_info = graph.steiner_info(base_edge_id);

                for (unsigned short i = 0; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                    steiner_graph::node_id_type const destination = {base_edge_id, i};
                    coordinate_t destination_coordinate = graph.node(destination).coordinates;

                    if (angle(from_other_edge_coordinate, source_coordinate, source_coordinate, destination_coordinate) < max_angle
                    || angle(from_coordinate, source_coordinate, source_coordinate, destination_coordinate) < max_angle) {
                        out.emplace_back(destination, node_id, 0);
                        destination_coordinates.emplace_back(destination_coordinate);
                    }
                }
            }
        }
    }


    template<typename NodeCostPair>
    void from_face_crossing_segment(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const& node_id = node.node;
        auto const& predecessor_id = node.predecessor;

        // get triangles that have not been visited yet
        auto &&triangles = graph.base_polyhedron().edge_faces(node_id.edge);
        unsigned char triangle_first = 0;
        unsigned char triangle_last = 2;

        // if this node is reached via a face crossing segment, only use edges in next face
        if (predecessor_id.edge != node_id.edge) [[likely]] {
            auto visited_triangles = graph.base_polyhedron().edge_faces(predecessor_id.edge);

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
                    steiner_graph::node_id_type const destination = {base_edge_id, i};
                    coordinate_t const destination_coordinate = graph.node(destination).coordinates;
                    if (angle(predecessor_coordinate, source_coordinate, source_coordinate, destination_coordinate) <
                        max_angle) [[unlikely]]
                        break;
                }

                for (; i < destination_steiner_info.node_count - 1; ++i) [[likely]] {
                    steiner_graph::node_id_type const destination = {base_edge_id, i};
                    coordinate_t destination_coordinate = graph.node(destination).coordinates;

                    if (angle(predecessor_coordinate, source_coordinate, source_coordinate, destination_coordinate) >
                        max_angle) [[likely]]
                        break;

                    out.emplace_back(destination, node_id, 0);
                    destination_coordinates.emplace_back(destination_coordinate);
                }
            }
        }
    }
public:
    pruned_steiner_neighbors(Graph const &graph, Labels const& labels) : graph{graph}, labels{labels} {}

    template<typename... Args>
    pruned_steiner_neighbors(Graph const &graph, Labels const& labels, Args... args) : graph{graph}, labels{labels} {}

    // TODO: prune according to paper section 2.4
    template<typename NodeCostPair>
    void operator()(NodeCostPair const &node, std::vector<NodeCostPair> &out) {
        auto const& __node_id = node.node;
        auto const& __reached_from = node.predecessor;

        const auto &&_steiner_info = graph.steiner_info(__node_id.edge);


        // for angle computations
        source_coordinate = graph.node(__node_id).coordinates;
        predecessor_coordinate = graph.node(__reached_from).coordinates;

        destination_coordinates.clear();

        assert(!is_none(__node_id));

        // Case 1
        if (graph.is_base_node(__node_id)) [[unlikely]]
            from_base_node(node, out);

        // Case 2: TODO make ε spanners
        if (__node_id.steiner_index == 1 || __node_id.steiner_index == _steiner_info.node_count - 1)
            from_base_neighboring_node(node, out);

        // Case 3.1: TODO make ε spanners
        if (__node_id.edge == __reached_from.edge)
            from_edge_using_segment(node, out);

        // Case 3.2
        if (__node_id.edge != __reached_from.edge)
            from_face_crossing_segment(node, out);

        // compute distances (can be vectorized)
        for (size_t e = 0; e < out.size(); ++e) [[likely]] {
            out[e].distance = distance(source_coordinate, destination_coordinates[e]) + node.distance;
        }
    }
};
